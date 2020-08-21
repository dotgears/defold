// Copyright 2020 The Defold Foundation
// Licensed under the Defold License version 1.0 (the "License"); you may not use
// this file except in compliance with the License.
//
// You may obtain a copy of the License, together with FAQs at
// https://www.defold.com/license
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include <sys/types.h>
#include <sys/stat.h>

#ifdef __linux__
#include <limits.h>
#elif defined (__MACH__)
#include <sys/param.h>
#endif

#if defined(_WIN32)
#include <malloc.h>
#define alloca(_SIZE) _alloca(_SIZE)
#endif

#include <dlib/dstrings.h>
#include <dlib/crypt.h>
#include <dlib/hash.h>
#include <dlib/hashtable.h>
#include <dlib/log.h>
#include <dlib/http_client.h>
#include <dlib/http_cache.h>
#include <dlib/http_cache_verify.h>
#include <dlib/math.h>
#include <dlib/memory.h>
#include <dlib/uri.h>
#include <dlib/path.h>
#include <dlib/profile.h>
#include <dlib/message.h>
#include <dlib/sys.h>
#include <dlib/time.h>
#include <dlib/mutex.h>

#include "resource.h"
#include "resource_ddf.h"
#include "resource_private.h"

/*
 * TODO:
 *
 *  - Resources could be loaded twice if canonical path is different for equivalent files.
 *    We could use realpath or similar function but we want to avoid file accesses when converting
 *    a canonical path to hash value. This functionality is used in the GetDescriptor function.
 *
 *  - If GetCanonicalPath exceeds RESOURCE_PATH_MAX PATH_TOO_LONG should be returned.
 *
 *  - Handle out of resources. Eg Hashtables full.
 */

extern dmDDF::Descriptor dmLiveUpdateDDF_ManifestFile_DESCRIPTOR;

namespace dmResource
{
const int DEFAULT_BUFFER_SIZE = 1024 * 1024;

#define RESOURCE_SOCKET_NAME "@resource"
#define LIVEUPDATE_MANIFEST_FILENAME "liveupdate.dmanifest"
#define LIVEUPDATE_BUNDLE_VER_FILENAME "bundle.ver"

const char* MAX_RESOURCES_KEY = "resource.max_resources";

struct ResourceReloadedCallbackPair
{
    ResourceReloadedCallback    m_Callback;
    void*                       m_UserData;
};

struct SResourceFactory
{
    // TODO: Arg... budget. Two hash-maps. Really necessary?
    dmHashTable<uint64_t, SResourceDescriptor>*  m_Resources;
    dmHashTable<uintptr_t, uint64_t>*            m_ResourceToHash;
    // Only valid if RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT is set
    // Used for reloading of resources
    dmHashTable<uint64_t, const char*>*          m_ResourceHashToFilename;
    // Only valid if RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT is set
    dmArray<ResourceReloadedCallbackPair>*       m_ResourceReloadedCallbacks;
    SResourceType                                m_ResourceTypes[MAX_RESOURCE_TYPES];
    uint32_t                                     m_ResourceTypesCount;

    // Guard for anything that touches anything that could be shared
    // with GetRaw (used for async threaded loading). Liveupdate, HttpClient, m_Buffer
    // m_BuiltinsManifest, m_Manifest
    dmMutex::HMutex                              m_LoadMutex;

    // dmResource::Get recursion depth
    uint32_t                                     m_RecursionDepth;
    // List of resources currently in dmResource::Get call-stack
    dmArray<const char*>                         m_GetResourceStack;

    dmMessage::HSocket                           m_Socket;

    dmURI::Parts                                 m_UriParts;
    dmHttpClient::HClient                        m_HttpClient;
    dmHttpCache::HCache                          m_HttpCache;
    LoadBufferType*                              m_HttpBuffer;

    dmArray<char>                                m_Buffer;

    // HTTP related state
    // Total number bytes loaded in current GET-request
    int32_t                                      m_HttpContentLength;
    uint32_t                                     m_HttpTotalBytesStreamed;
    int                                          m_HttpStatus;
    Result                                       m_HttpFactoryResult;

    // Manifest for builtin resources
    Manifest*                                    m_BuiltinsManifest;

    // Resource manifest
    Manifest*                                    m_Manifest;
    void*                                        m_ArchiveMountInfo;

    uint8_t                                      m_UseLiveUpdate : 1;
};

SResourceType* FindResourceType(SResourceFactory* factory, const char* extension)
{
    for (uint32_t i = 0; i < factory->m_ResourceTypesCount; ++i)
    {
        SResourceType* rt = &factory->m_ResourceTypes[i];
        if (strcmp(extension, rt->m_Extension) == 0)
        {
            return rt;
        }
    }
    return 0;
}

// TODO: Test this...
uint32_t GetCanonicalPathFromBase(const char* base_dir, const char* relative_dir, char* buf)
{
    dmSnPrintf(buf, RESOURCE_PATH_MAX, "%s/%s", base_dir, relative_dir);

    char* source = buf;
    char* dest = buf;
    char last_c = 0;
    while (*source != 0)
    {
        char c = *source;
        if (c != '/' || (c == '/' && last_c != '/'))
            *dest++ = c;

        last_c = c;
        ++source;
    }
    *dest = '\0';
    return (uint32_t)(dest - buf);
}

uint32_t GetCanonicalPath(const char* relative_dir, char* buf)
{
    return GetCanonicalPathFromBase("", relative_dir, buf);
}

Result CheckSuppliedResourcePath(const char* name)
{
    if (name[0] == 0)
    {
        dmLogError("Empty resource path");
        return RESULT_RESOURCE_NOT_FOUND;
    }
    if (name[0] != '/')
    {
        dmLogError("Resource path is not absolute (%s)", name);
        return RESULT_RESOURCE_NOT_FOUND;
    }
    return RESULT_OK;
}

void SetDefaultNewFactoryParams(struct NewFactoryParams* params)
{
    params->m_MaxResources = 1024;
    params->m_Flags = RESOURCE_FACTORY_FLAGS_EMPTY;

    params->m_ArchiveManifest.m_Data = 0;
    params->m_ArchiveManifest.m_Size = 0;
    params->m_ArchiveIndex.m_Data = 0;
    params->m_ArchiveIndex.m_Size = 0;
    params->m_ArchiveData.m_Data = 0;
    params->m_ArchiveData.m_Size = 0;
}

static void HttpHeader(dmHttpClient::HResponse response, void* user_data, int status_code, const char* key, const char* value)
{
    SResourceFactory* factory = (SResourceFactory*) user_data;
    factory->m_HttpStatus = status_code;

    if (dmStrCaseCmp(key, "Content-Length") == 0)
    {
        factory->m_HttpContentLength = strtol(value, 0, 10);
        if (factory->m_HttpContentLength < 0) {
            dmLogError("Content-Length negative (%d)", factory->m_HttpContentLength);
        } else {
            if (factory->m_HttpBuffer->Capacity() < (uint32_t)factory->m_HttpContentLength) {
                factory->m_HttpBuffer->SetCapacity(factory->m_HttpContentLength);
            }
            factory->m_HttpBuffer->SetSize(0);
        }
    }
}

static void HttpContent(dmHttpClient::HResponse, void* user_data, int status_code, const void* content_data, uint32_t content_data_size)
{
    SResourceFactory* factory = (SResourceFactory*) user_data;
    (void) status_code;

    if (!content_data && content_data_size)
    {
        factory->m_HttpBuffer->SetSize(0);
        return;
    }

    // We must set http-status here. For direct cached result HttpHeader is not called.
    factory->m_HttpStatus = status_code;

    if (factory->m_HttpBuffer->Remaining() < content_data_size) {
        uint32_t diff = content_data_size - factory->m_HttpBuffer->Remaining();
        // NOTE: Resizing the the array can be inefficient but sometimes we don't know the actual size, i.e. when "Content-Size" isn't set
        factory->m_HttpBuffer->OffsetCapacity(diff + 1024 * 1024);
    }

    factory->m_HttpBuffer->PushArray((const char*) content_data, content_data_size);
    factory->m_HttpTotalBytesStreamed += content_data_size;
}

Manifest* GetManifest(HFactory factory)
{
    return factory->m_Manifest;
}

uint32_t HashLength(dmLiveUpdateDDF::HashAlgorithm algorithm)
{
    const uint32_t bitlen[5] = { 0U, 128U, 160U, 256U, 512U };
    return bitlen[(int) algorithm] / 8U;
}

void BytesToHexString(const uint8_t* byte_buf, uint32_t byte_buf_len, char* out_buf, uint32_t out_len)
{
    if (out_buf != NULL && out_len > 0)
    {
        uint32_t out_len_cond = (out_len + 1) / 2;
        out_buf[0] = 0x0;
        for (uint32_t i = 0; i < byte_buf_len; ++i)
        {
            char current[3];
            dmSnPrintf(current, 3, "%02x", byte_buf[i]);

            if (i < out_len_cond)
                strncat(out_buf, current, 1);
            if (i+1 < out_len_cond)
                strncat(out_buf, current+1, 1);
            else
                break;
        }
    }
}

Result StoreManifest(Manifest* manifest)
{
    char app_support_path[DMPATH_MAX_PATH];
    char id_buf[MANIFEST_PROJ_ID_LEN]; // String repr. of project id SHA1 hash
    char manifest_file_path[DMPATH_MAX_PATH];
    char manifest_tmp_file_path[DMPATH_MAX_PATH];
    BytesToHexString(manifest->m_DDFData->m_Header.m_ProjectIdentifier.m_Data.m_Data, HashLength(dmLiveUpdateDDF::HASH_SHA1), id_buf, MANIFEST_PROJ_ID_LEN);

    dmSys::Result support_path_result = dmSys::GetApplicationSupportPath(id_buf, app_support_path, DMPATH_MAX_PATH);
    if (support_path_result != dmSys::RESULT_OK)
    {
        dmLogError("Failed get application support path for \"%s\", result = %i", id_buf, support_path_result);
        return RESULT_IO_ERROR;
    }

    dmPath::Concat(app_support_path, LIVEUPDATE_MANIFEST_FILENAME, manifest_file_path, DMPATH_MAX_PATH);
    dmStrlCpy(manifest_tmp_file_path, manifest_file_path, DMPATH_MAX_PATH);
    dmSnPrintf(manifest_tmp_file_path, sizeof(manifest_tmp_file_path), "%s.tmp", manifest_file_path);
    // write to tempfile, if successful move/rename and then delete tmpfile
    dmDDF::Result ddf_result = dmDDF::SaveMessageToFile(manifest->m_DDF, dmLiveUpdateDDF::ManifestFile::m_DDFDescriptor, manifest_tmp_file_path);
    if (ddf_result != dmDDF::RESULT_OK)
    {
        dmLogError("Failed storing manifest to file, result: %i", ddf_result);
        return RESULT_DDF_ERROR;
    }
    dmSys::Result sys_result = dmSys::RenameFile(manifest_file_path, manifest_tmp_file_path);
    if (sys_result != dmSys::RESULT_OK)
    {
        return RESULT_IO_ERROR;
    }
    return RESULT_OK;
}

Result LoadArchiveIndex(const char* bundle_dir, HFactory factory)
{
    Result result = RESULT_OK;
    const uint32_t manifest_extension_length = strlen("dmanifest");
    const uint32_t index_extension_length = strlen("arci");

    char archive_index_path[DMPATH_MAX_PATH];
    char archive_resource_path[DMPATH_MAX_PATH];
    char liveupdate_index_path[DMPATH_MAX_PATH];
    char app_support_path[DMPATH_MAX_PATH];
    char id_buf[MANIFEST_PROJ_ID_LEN]; // String repr. of project id SHA1 hash

    dmStrlCpy(archive_resource_path, bundle_dir, strlen(bundle_dir) - manifest_extension_length + 1);
    dmStrlCat(archive_resource_path, "arcd", DMPATH_MAX_PATH);
    // derive path to arci file from path to arcd file
    dmStrlCpy(archive_index_path, archive_resource_path, DMPATH_MAX_PATH);
    archive_index_path[strlen(archive_index_path) - 1] = 'i';

    BytesToHexString(factory->m_Manifest->m_DDFData->m_Header.m_ProjectIdentifier.m_Data.m_Data, HashLength(dmLiveUpdateDDF::HASH_SHA1), id_buf, MANIFEST_PROJ_ID_LEN);

    bool luIndexExists = false;
    if (factory->m_UseLiveUpdate)
    {
        dmSys::Result support_path_result = dmSys::GetApplicationSupportPath(id_buf, app_support_path, DMPATH_MAX_PATH);
        if (support_path_result != dmSys::RESULT_OK)
        {
            dmLogError("Failed get application support path for \"%s\", result = %i", id_buf, support_path_result);
            return RESULT_IO_ERROR;
        }

        dmPath::Concat(app_support_path, "liveupdate.arci", liveupdate_index_path, DMPATH_MAX_PATH);
        struct stat file_stat;
        luIndexExists = stat(liveupdate_index_path, &file_stat) == 0;
    }

    if (!luIndexExists)
    {
        result = MountArchiveInternal(archive_index_path, archive_resource_path, 0x0, &factory->m_Manifest->m_ArchiveIndex, &factory->m_ArchiveMountInfo);
    }
    else // If a liveupdate index exists, use that one instead
    {
        char liveupdate_resource_path[DMPATH_MAX_PATH];
        dmStrlCpy(liveupdate_resource_path, liveupdate_index_path, strlen(liveupdate_index_path) - index_extension_length + 1);
        dmStrlCat(liveupdate_resource_path, "arcd", DMPATH_MAX_PATH);
        // Check if any liveupdate resources were stored last time engine was running
        char temp_archive_index_path[DMPATH_MAX_PATH];
        dmStrlCpy(temp_archive_index_path, liveupdate_index_path, strlen(liveupdate_index_path)+1);
        dmStrlCat(temp_archive_index_path, ".tmp", DMPATH_MAX_PATH); // check for liveupdate.arci.tmp
        struct stat file_stat;
        bool luTempIndexExists = stat(temp_archive_index_path, &file_stat) == 0;
        if (luTempIndexExists)
        {
            dmSys::Result moveResult = dmSys::RenameFile(liveupdate_index_path, temp_archive_index_path);

            if (moveResult != dmSys::RESULT_OK)
            {
                // The recently added resources will not be available if we proceed after this point
                dmLogError("Fail to load liveupdate index data (%i).", moveResult);
                return RESULT_IO_ERROR;
            }
            dmSys::Unlink(temp_archive_index_path);
        }
        result = MountArchiveInternal(liveupdate_index_path, archive_resource_path, liveupdate_resource_path, &factory->m_Manifest->m_ArchiveIndex, &factory->m_ArchiveMountInfo);
        if (result != RESULT_OK)
        {
            dmLogError("Failed to mount archive, result = %i", result);
            return RESULT_IO_ERROR;
        }
        int archive_id_cmp = dmResourceArchive::CmpArchiveIdentifier(factory->m_Manifest->m_ArchiveIndex, factory->m_Manifest->m_DDF->m_ArchiveIdentifier.m_Data, factory->m_Manifest->m_DDF->m_ArchiveIdentifier.m_Count);
        if (archive_id_cmp != 0)
        {
            dmResourceArchive::Result reload_res = ReloadBundledArchiveIndex(archive_index_path, archive_resource_path, liveupdate_index_path, liveupdate_resource_path, factory->m_Manifest->m_ArchiveIndex, factory->m_ArchiveMountInfo);

            if (reload_res != dmResourceArchive::RESULT_OK)
            {
                dmLogError("Failed to reload liveupdate index with bundled index, result = %i", reload_res);
                return RESULT_IO_ERROR;
            }
        }
	}

    return result;
}

Result ManifestLoadMessage(uint8_t* manifest_msg_buf, uint32_t size, dmResource::Manifest*& out_manifest)
{
    // Read from manifest resource
    dmDDF::Result result = dmDDF::LoadMessage(manifest_msg_buf, size, dmLiveUpdateDDF::ManifestFile::m_DDFDescriptor, (void**) &out_manifest->m_DDF);
    if (result != dmDDF::RESULT_OK)
    {
        dmLogError("Failed to parse Manifest (%i)", result);
        return RESULT_DDF_ERROR;
    }

    // Read data blob from ManifestFile into ManifestData message
    result = dmDDF::LoadMessage(out_manifest->m_DDF->m_Data.m_Data, out_manifest->m_DDF->m_Data.m_Count, dmLiveUpdateDDF::ManifestData::m_DDFDescriptor, (void**) &out_manifest->m_DDFData);
    if (result != dmDDF::RESULT_OK)
    {
        dmLogError("Failed to parse Manifest data (%i)", result);
        dmDDF::FreeMessage(out_manifest->m_DDF);
        out_manifest->m_DDF = 0x0;
        return RESULT_DDF_ERROR;
    }
    if (out_manifest->m_DDFData->m_Header.m_MagicNumber != MANIFEST_MAGIC_NUMBER)
    {
        dmLogError("Manifest format mismatch (expected '%x', actual '%x')",
            MANIFEST_MAGIC_NUMBER, out_manifest->m_DDFData->m_Header.m_MagicNumber);
        dmDDF::FreeMessage(out_manifest->m_DDFData);
        dmDDF::FreeMessage(out_manifest->m_DDF);
        out_manifest->m_DDFData = 0x0;
        out_manifest->m_DDF = 0x0;
        return RESULT_FORMAT_ERROR;
    }

    if (out_manifest->m_DDFData->m_Header.m_Version != MANIFEST_VERSION)
    {
        dmLogError("Manifest version mismatch (expected '%i', actual '%i')",
            dmResourceArchive::VERSION, out_manifest->m_DDFData->m_Header.m_Version);
        dmDDF::FreeMessage(out_manifest->m_DDFData);
        dmDDF::FreeMessage(out_manifest->m_DDF);
        out_manifest->m_DDFData = 0x0;
        out_manifest->m_DDF = 0x0;
        return RESULT_VERSION_MISMATCH;
    }

    return RESULT_OK;
}

static Result LoadManifest(const char* manifestPath, HFactory factory)
{
    uint32_t manifestLength = 0;
    uint8_t* manifestBuffer = 0x0;

    uint32_t dummy_file_size = 0;
    dmSys::ResourceSize(manifestPath, &manifestLength);
    dmMemory::AlignedMalloc((void**)&manifestBuffer, 16, manifestLength);
    assert(manifestBuffer);
    dmSys::Result sysResult = dmSys::LoadResource(manifestPath, manifestBuffer, manifestLength, &dummy_file_size);

    if (sysResult != dmSys::RESULT_OK)
    {
        dmLogError("Failed to read Manifest (%i)", sysResult);
        dmMemory::AlignedFree(manifestBuffer);
        return RESULT_IO_ERROR;
    }

    Result result = ManifestLoadMessage(manifestBuffer, manifestLength, factory->m_Manifest);
    dmMemory::AlignedFree(manifestBuffer);

    return result;
}

// Load manifest at specified manifest_path instead of from bundle
Result LoadExternalManifest(const char* manifest_path, HFactory factory)
{
// Android differs in storage for resources in local storage compared to bundled resources
#if !defined(__ANDROID__)
        return LoadManifest(manifest_path, factory);
#else
    uint32_t manifest_len = 0;
    uint8_t* manifest_buf = 0x0;

    Result map_res = MountManifest(manifest_path, (void*&)manifest_buf, manifest_len);
    assert(manifest_buf);
    if (map_res != RESULT_OK)
    {
        UnmountManifest((void*&)manifest_buf, manifest_len);
        return RESULT_IO_ERROR;
    }

    Result result = ManifestLoadMessage(manifest_buf, manifest_len, factory->m_Manifest);
    UnmountManifest((void*&)manifest_buf, manifest_len);

    return result;
#endif
}

Result HashCompare(const uint8_t* digest, uint32_t len, const uint8_t* expected_digest, uint32_t expected_len)
{
    if (expected_len != len)
    {
        dmLogError("Length mismatch in hash comparison. Expected %u, got %u", expected_len, len);
        return RESULT_FORMAT_ERROR;
    }
    for (uint32_t i = 0; i < expected_len; ++i)
    {
        if (expected_digest[i] != digest[i])
        {
            dmLogError("Byte mismatch in decrypted manifest signature. Different keys used for signing?");
            return RESULT_FORMAT_ERROR;
        }
    }
    return RESULT_OK;
}

Result DecryptSignatureHash(Manifest* manifest, const uint8_t* pub_key_buf, uint32_t pub_key_len, uint8_t** out_digest, uint32_t* out_digest_len)
{
    uint8_t* signature = manifest->m_DDF->m_Signature.m_Data;
    uint32_t signature_len = manifest->m_DDF->m_Signature.m_Count;
    uint32_t signature_hash_len = HashLength(manifest->m_DDFData->m_Header.m_SignatureHashAlgorithm);

    dmCrypt::Result r = dmCrypt::Decrypt(pub_key_buf, pub_key_len, signature, signature_len, out_digest, out_digest_len);
    if (r != dmCrypt::RESULT_OK) {
        return RESULT_INVALID_DATA;
    }
    return RESULT_OK;
}

Result VerifyManifestHash(HFactory factory, Manifest* manifest, const uint8_t* expected_digest, uint32_t expected_len)
{
    if (strcmp(factory->m_UriParts.m_Scheme, "dmanif") != 0)
    {
        dmLogWarning("Skipping manifest verification, resources are loaded with scheme: '%s' and not from manifest.", factory->m_UriParts.m_Scheme);
        return RESULT_NOT_SUPPORTED;
    }

    Result res = RESULT_OK;
    char public_key_path[DMPATH_MAX_PATH];
    char game_dir[DMPATH_MAX_PATH];
    uint32_t pub_key_size = 0, hash_decrypted_len = 0, out_resource_size = 0;
    uint8_t* pub_key_buf = 0x0;
    uint8_t* hash_decrypted = 0x0;

    // Load public key
    dmPath::Dirname(factory->m_UriParts.m_Path, game_dir, DMPATH_MAX_PATH);
    dmPath::Concat(game_dir, "game.public.der", public_key_path, DMPATH_MAX_PATH);
    dmSys::Result sys_res = dmSys::ResourceSize(public_key_path, &pub_key_size);
    if (sys_res != dmSys::RESULT_OK)
    {
        dmLogError("Failed to get size of public key for manifest verification (%i) at path: %s", sys_res, public_key_path);
        free(pub_key_buf);
        return RESULT_IO_ERROR;
    }
    pub_key_buf = (uint8_t*)malloc(pub_key_size);
    assert(pub_key_buf);
    sys_res = dmSys::LoadResource(public_key_path, pub_key_buf, pub_key_size, &out_resource_size);

    if (sys_res != dmSys::RESULT_OK)
    {
        dmLogError("Failed to load public key for manifest verification (%i) at path: %s", sys_res, public_key_path);
        free(pub_key_buf);
        return RESULT_IO_ERROR;
    }

    if (out_resource_size != pub_key_size)
    {
        dmLogError("Failed to load public key for manifest verification at path: %s, tried reading %d bytes, got %d bytes", public_key_path, pub_key_size, out_resource_size);
        free(pub_key_buf);
        return RESULT_IO_ERROR;
    }

    res = DecryptSignatureHash(manifest, pub_key_buf, pub_key_size, &hash_decrypted, &hash_decrypted_len);
    if (res != RESULT_OK)
    {
        return res;
    }
    res = HashCompare((const uint8_t*)hash_decrypted, hash_decrypted_len, expected_digest, expected_len);

    free(hash_decrypted);
    free(pub_key_buf);
    return res;
}

Result NewArchiveIndexWithResource(Manifest* manifest, const uint8_t* hashDigest, uint32_t hashDigestLength, const dmResourceArchive::LiveUpdateResource* resource, const char* proj_id, dmResourceArchive::HArchiveIndex& out_new_index)
{
    dmResourceArchive::Result result = dmResourceArchive::NewArchiveIndexWithResource(manifest->m_ArchiveIndex, hashDigest, hashDigestLength, resource, proj_id, out_new_index);
    return (result == dmResourceArchive::RESULT_OK) ? RESULT_OK : RESULT_INVAL;
}

Result BundleVersionValid(const Manifest* manifest, const char* bundle_ver_path)
{
    Result result = RESULT_OK;
    struct stat file_stat;
    bool bundle_ver_exists = stat(bundle_ver_path, &file_stat) == 0;

    uint8_t* signature = manifest->m_DDF->m_Signature.m_Data;
    uint32_t signature_len = manifest->m_DDF->m_Signature.m_Count;
    if (bundle_ver_exists)
    {
        FILE* bundle_ver = fopen(bundle_ver_path, "rb");
        uint8_t* buf = (uint8_t*)alloca(signature_len);
        fread(buf, 1, signature_len, bundle_ver);
        fclose(bundle_ver);
        if (memcmp(buf, signature, signature_len) != 0)
        {
            // Bundle has changed, local liveupdate manifest no longer valid.
            result = RESULT_VERSION_MISMATCH;
        }
    }
    else
    {
        // Take bundled manifest signature and write to 'bundle_ver' file
        FILE* bundle_ver = fopen(bundle_ver_path, "wb");
        size_t bytes_written = fwrite(signature, 1, signature_len, bundle_ver);
        if (bytes_written != signature_len)
        {
            dmLogWarning("Failed to write bundle version to file, wrote %u bytes out of %u bytes.", (uint32_t)bytes_written, signature_len);
        }
        fclose(bundle_ver);
        result = RESULT_OK;
    }

    return result;
}

HFactory NewFactory(NewFactoryParams* params, const char* uri)
{
    dmMessage::HSocket socket = 0;

    dmMessage::Result mr = dmMessage::NewSocket(RESOURCE_SOCKET_NAME, &socket);
    if (mr != dmMessage::RESULT_OK)
    {
        dmLogFatal("Unable to create resource socket: %s (%d)", RESOURCE_SOCKET_NAME, mr);
        return 0;
    }

    SResourceFactory* factory = new SResourceFactory;
    memset(factory, 0, sizeof(*factory));
    factory->m_Socket = socket;
    factory->m_UseLiveUpdate = params->m_Flags & RESOURCE_FACTORY_FLAGS_LIVE_UPDATE ? 1 : 0;

    dmURI::Result uri_result = dmURI::Parse(uri, &factory->m_UriParts);
    if (uri_result != dmURI::RESULT_OK)
    {
        dmLogError("Unable to parse uri: %s", uri);
        dmMessage::DeleteSocket(socket);
        delete factory;
        return 0;
    }

    dmDNS::HChannel dns_channel;
    dmDNS::NewChannel(&dns_channel);

    factory->m_HttpBuffer = 0;
    factory->m_HttpClient = 0;
    factory->m_HttpCache = 0;
    if (strcmp(factory->m_UriParts.m_Scheme, "http") == 0 || strcmp(factory->m_UriParts.m_Scheme, "https") == 0)
    {
        factory->m_HttpCache = 0;
        if (params->m_Flags & RESOURCE_FACTORY_FLAGS_HTTP_CACHE)
        {
            dmHttpCache::NewParams cache_params;
            char path[1024];
            dmSys::Result sys_result = dmSys::GetApplicationSupportPath("defold", path, sizeof(path));
            if (sys_result == dmSys::RESULT_OK)
            {
                // NOTE: The other http-service cache is called /http-cache
                dmStrlCat(path, "/cache", sizeof(path));
                cache_params.m_Path = path;
                dmHttpCache::Result cache_r = dmHttpCache::Open(&cache_params, &factory->m_HttpCache);
                if (cache_r != dmHttpCache::RESULT_OK)
                {
                    dmLogWarning("Unable to open http cache (%d)", cache_r);
                }
                else
                {
                    dmHttpCacheVerify::Result verify_r = dmHttpCacheVerify::VerifyCache(factory->m_HttpCache, &factory->m_UriParts, dns_channel, 60 * 60 * 24 * 5); // 5 days
                    // Http-cache batch verification might be unsupported
                    // We currently does not have support for batch validation in the editor http-server
                    // Batch validation was introduced when we had remote branch and latency problems
                    if (verify_r != dmHttpCacheVerify::RESULT_OK && verify_r != dmHttpCacheVerify::RESULT_UNSUPPORTED)
                    {
                        dmLogWarning("Cache validation failed (%d)", verify_r);
                    }

                    dmHttpCache::SetConsistencyPolicy(factory->m_HttpCache, dmHttpCache::CONSISTENCY_POLICY_TRUST_CACHE);
                }
            }
            else
            {
                dmLogWarning("Unable to locate application support path for \"%s\": (%d)", "defold", sys_result);
            }
        }

        dmHttpClient::NewParams http_params;
        http_params.m_HttpHeader = &HttpHeader;
        http_params.m_HttpContent = &HttpContent;
        http_params.m_Userdata = factory;
        http_params.m_HttpCache = factory->m_HttpCache;
        http_params.m_DNSChannel = dns_channel;
        factory->m_HttpClient = dmHttpClient::New(&http_params, factory->m_UriParts.m_Hostname, factory->m_UriParts.m_Port, strcmp(factory->m_UriParts.m_Scheme, "https") == 0);
        if (!factory->m_HttpClient)
        {
            dmLogError("Invalid URI: %s", uri);
            dmMessage::DeleteSocket(socket);
            dmDNS::DeleteChannel(dns_channel);
            delete factory;
            return 0;
        }
    }
    else if (strcmp(factory->m_UriParts.m_Scheme, "file") == 0
#if defined(__NX__)
        || strcmp(factory->m_UriParts.m_Scheme, "data") == 0
        || strcmp(factory->m_UriParts.m_Scheme, "host") == 0
#endif
        )
    {
        // Ok
    }
    else if (strcmp(factory->m_UriParts.m_Scheme, "dmanif") == 0)
    {
        factory->m_Manifest = new Manifest();
        factory->m_ArchiveMountInfo = 0x0;

        char* manifest_path = factory->m_UriParts.m_Path;
        Result r = LoadManifest(manifest_path, factory);

        // Nothing to do to recover here
        if (r != RESULT_OK)
        {
            dmLogError("Unable to load bundled manifest: %s with result: %i.", factory->m_UriParts.m_Path, r);
            dmMessage::DeleteSocket(socket);
            delete factory->m_Manifest;
            delete factory;
            return 0;
        }

        // Check if liveupdate manifest exists. If it does, try to load that one instead

        if (factory->m_UseLiveUpdate)
        {
            char app_support_path[DMPATH_MAX_PATH];
            char lu_manifest_file_path[DMPATH_MAX_PATH];
            char id_buf[MANIFEST_PROJ_ID_LEN]; // String repr. of project id SHA1 hash
            BytesToHexString(factory->m_Manifest->m_DDFData->m_Header.m_ProjectIdentifier.m_Data.m_Data, HashLength(dmLiveUpdateDDF::HASH_SHA1), id_buf, MANIFEST_PROJ_ID_LEN);
            dmSys::Result support_path_result = dmSys::GetApplicationSupportPath(id_buf, app_support_path, DMPATH_MAX_PATH);
            if (support_path_result != dmSys::RESULT_OK)
            {
                dmLogError("Failed get application support path for \"%s\", result = %i", id_buf, support_path_result);
                r = RESULT_IO_ERROR;
            }
            else
            {
                dmPath::Concat(app_support_path, LIVEUPDATE_MANIFEST_FILENAME, lu_manifest_file_path, DMPATH_MAX_PATH);
                struct stat file_stat;
                bool lu_manifest_exists = stat(lu_manifest_file_path, &file_stat) == 0;
                if (lu_manifest_exists)
                {
                    // Check if bundle has changed (e.g. app upgraded)
                    char bundle_ver_path[DMPATH_MAX_PATH];
                    dmPath::Concat(app_support_path, LIVEUPDATE_BUNDLE_VER_FILENAME, bundle_ver_path, DMPATH_MAX_PATH);

                    Result bundle_ver_valid = BundleVersionValid(factory->m_Manifest, bundle_ver_path);
                    if (bundle_ver_valid == RESULT_OK)
                    {
                        // Unload bundled manifest
                        dmDDF::FreeMessage(factory->m_Manifest->m_DDFData);
                        dmDDF::FreeMessage(factory->m_Manifest->m_DDF);
                        factory->m_Manifest->m_DDFData = 0x0;
                        factory->m_Manifest->m_DDF = 0x0;
                        // Load external liveupdate.manifest
                        r = LoadExternalManifest(lu_manifest_file_path, factory);
                        // Use liveupdate manifest if successfully loaded, otherwise fall back to bundled manifest
                        if (r == RESULT_OK)
                            manifest_path = lu_manifest_file_path;
                        else
                        {
                            dmLogWarning("Failed to load liveupdate manifest: %s with result: %i. Falling back to bundled manifest", lu_manifest_file_path, r);
                            LoadManifest(manifest_path, factory);
                        }
                    }
                    else
                    {
                        // Bundle version file exists from previous run, but signature does not match currently loaded bundled manifest.
                        // Unlink liveupdate.manifest and bundle_ver_path from filesystem and load bundled manifest instead.
                        dmSys::Unlink(bundle_ver_path);
                        dmSys::Unlink(lu_manifest_file_path);
                    }
                }
            }
        }

        r = LoadArchiveIndex(factory->m_UriParts.m_Path, factory);

        if (r == RESULT_OK)
        {
            // Only need factory->m_Manifest->m_DDFData from this point on, make sure we release unneeded message
            dmDDF::FreeMessage(factory->m_Manifest->m_DDF);
            factory->m_Manifest->m_DDF = 0x0;
        }
        else
        {
            dmLogError("Unable to load archive.");
        }

        if (r != RESULT_OK)
        {
            dmLogError("Failed to create factory %s with result %i.", factory->m_UriParts.m_Path, r);
            dmMessage::DeleteSocket(socket);
            dmDDF::FreeMessage(factory->m_Manifest->m_DDF);
            dmDDF::FreeMessage(factory->m_Manifest->m_DDFData);
            factory->m_Manifest->m_DDF = 0x0;
            factory->m_Manifest->m_DDFData = 0x0;
            delete factory->m_Manifest;
            delete factory;
            return 0;
        }
    }
    else
    {
        dmLogError("Invalid URI: %s", uri);
        dmMessage::DeleteSocket(socket);
        delete factory;
        return 0;
    }

    factory->m_ResourceTypesCount = 0;

    const uint32_t table_size = dmMath::Max(1u, (3 * params->m_MaxResources) / 4);
    factory->m_Resources = new dmHashTable<uint64_t, SResourceDescriptor>();
    factory->m_Resources->SetCapacity(table_size, params->m_MaxResources);

    factory->m_ResourceToHash = new dmHashTable<uintptr_t, uint64_t>();
    factory->m_ResourceToHash->SetCapacity(table_size, params->m_MaxResources);

    if (params->m_Flags & RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT)
    {
        factory->m_ResourceHashToFilename = new dmHashTable<uint64_t, const char*>();
        factory->m_ResourceHashToFilename->SetCapacity(table_size, params->m_MaxResources);

        factory->m_ResourceReloadedCallbacks = new dmArray<ResourceReloadedCallbackPair>();
        factory->m_ResourceReloadedCallbacks->SetCapacity(256);
    }
    else
    {
        factory->m_ResourceHashToFilename = 0;
        factory->m_ResourceReloadedCallbacks = 0;
    }

    if (params->m_ArchiveManifest.m_Size)
    {
        factory->m_BuiltinsManifest = new Manifest();
        dmDDF::Result res = dmDDF::LoadMessage(params->m_ArchiveManifest.m_Data, params->m_ArchiveManifest.m_Size, dmLiveUpdateDDF::ManifestFile::m_DDFDescriptor, (void**)&factory->m_BuiltinsManifest->m_DDF);

        if (res != dmDDF::RESULT_OK)
        {
            dmLogError("Failed to load builtins manifest, result: %u", res);
        }
        else
        {
            res = dmDDF::LoadMessage(factory->m_BuiltinsManifest->m_DDF->m_Data.m_Data, factory->m_BuiltinsManifest->m_DDF->m_Data.m_Count, dmLiveUpdateDDF::ManifestData::m_DDFDescriptor, (void**)&factory->m_BuiltinsManifest->m_DDFData);
            dmResourceArchive::WrapArchiveBuffer(params->m_ArchiveIndex.m_Data, params->m_ArchiveData.m_Data, 0x0, 0x0, 0x0, &factory->m_BuiltinsManifest->m_ArchiveIndex);
        }
    }

    factory->m_LoadMutex = dmMutex::New();
    return factory;
}

void DeleteFactory(HFactory factory)
{
    if (factory->m_Socket)
    {
        dmMessage::DeleteSocket(factory->m_Socket);
    }
    if (factory->m_HttpClient)
    {
        dmDNS::HChannel dns_channel = dmHttpClient::GetDNSChannel(factory->m_HttpClient);
        dmHttpClient::Delete(factory->m_HttpClient);
        dmDNS::DeleteChannel(dns_channel);
    }
    if (factory->m_HttpCache)
    {
        dmHttpCache::Close(factory->m_HttpCache);
    }
    if (factory->m_LoadMutex)
    {
        dmMutex::Delete(factory->m_LoadMutex);
    }
    if (factory->m_Manifest)
    {
        if (factory->m_Manifest->m_DDF)
        {
            dmDDF::FreeMessage(factory->m_Manifest->m_DDF);
            factory->m_Manifest->m_DDF = 0x0;
        }

        if (factory->m_Manifest->m_DDFData)
        {
            dmDDF::FreeMessage(factory->m_Manifest->m_DDFData);
            factory->m_Manifest->m_DDFData = 0x0;
        }

        if (factory->m_Manifest->m_ArchiveIndex)
        {
            if (factory->m_ArchiveMountInfo)
                UnmountArchiveInternal(factory->m_Manifest->m_ArchiveIndex, factory->m_ArchiveMountInfo);
            else
                dmResourceArchive::Delete(factory->m_Manifest->m_ArchiveIndex);
        }

        delete factory->m_Manifest;
    }

    ReleaseBuiltinsManifest(factory);

    delete factory->m_Resources;
    delete factory->m_ResourceToHash;
    if (factory->m_ResourceHashToFilename)
        delete factory->m_ResourceHashToFilename;
    if (factory->m_ResourceReloadedCallbacks)
        delete factory->m_ResourceReloadedCallbacks;
    delete factory;
}

static void Dispatch(dmMessage::Message* message, void* user_ptr)
{
    HFactory factory = (HFactory) user_ptr;

    if (message->m_Descriptor)
    {
        dmDDF::Descriptor* descriptor = (dmDDF::Descriptor*)message->m_Descriptor;
        if (descriptor == dmResourceDDF::Reload::m_DDFDescriptor)
        {
            // NOTE use offsets instead of reading via ddf message
            // Message: | offset to string-offsets | count | string offset | string offset | ... | string #1 | string #2 | ... |
            dmResourceDDF::Reload* reload_resources = (dmResourceDDF::Reload*) message->m_Data;
            uint32_t count = reload_resources->m_Resources.m_Count;
            uint8_t* str_offset_cursor = (uint8_t*)((uintptr_t)reload_resources + *(uint32_t*)reload_resources);
            for (uint32_t i = 0; i < count; ++i)
            {
                const char* resource = (const char *) (uintptr_t)reload_resources + *(str_offset_cursor + i * sizeof(uint64_t));
                SResourceDescriptor* resource_descriptor;
                ReloadResource(factory, resource, &resource_descriptor);
            }
        }
        else
        {
            dmLogError("Unknown message '%s' sent to socket '%s'.\n", descriptor->m_Name, RESOURCE_SOCKET_NAME);
        }
    }
    else
    {
        dmLogError("Only system messages can be sent to the '%s' socket.\n", RESOURCE_SOCKET_NAME);
    }
}

void UpdateFactory(HFactory factory)
{
    dmMessage::Dispatch(factory->m_Socket, &Dispatch, factory);
}

Result RegisterType(HFactory factory,
                           const char* extension,
                           void* context,
                           FResourcePreload preload_function,
                           FResourceCreate create_function,
                           FResourcePostCreate post_create_function,
                           FResourceDestroy destroy_function,
                           FResourceRecreate recreate_function)
{
    if (factory->m_ResourceTypesCount == MAX_RESOURCE_TYPES)
        return RESULT_OUT_OF_RESOURCES;

    // Dots not allowed in extension
    if (strrchr(extension, '.') != 0)
        return RESULT_INVAL;

    if (create_function == 0 || destroy_function == 0)
        return RESULT_INVAL;

    if (FindResourceType(factory, extension) != 0)
        return RESULT_ALREADY_REGISTERED;

    SResourceType resource_type;
    resource_type.m_ExtensionHash = dmHashString64(extension);
    resource_type.m_Extension = extension;
    resource_type.m_Context = context;
    resource_type.m_PreloadFunction = preload_function;
    resource_type.m_CreateFunction = create_function;
    resource_type.m_PostCreateFunction = post_create_function;
    resource_type.m_DestroyFunction = destroy_function;
    resource_type.m_RecreateFunction = recreate_function;

    factory->m_ResourceTypes[factory->m_ResourceTypesCount++] = resource_type;

    return RESULT_OK;
}

// Finds the specific entry in a sorted list of entries
static int FindEntryIndex(const Manifest* manifest, dmhash_t path_hash)
{
    uint32_t entry_count = manifest->m_DDFData->m_Resources.m_Count;
    dmLiveUpdateDDF::ResourceEntry* entries = manifest->m_DDFData->m_Resources.m_Data;

    // Use divide and conquer
    int first = 0;
    int last = entry_count-1;
    while (first <= last)
    {
        int mid = first + (last - first) / 2;
        uint64_t h = entries[mid].m_UrlHash;

        if (h == path_hash)
        {
            return mid;
        }
        else if (h > path_hash)
        {
            last = mid - 1;
        }
        else if (h < path_hash)
        {
            first = mid + 1;
        }
    }
    return -1;
}

Result VerifyResourcesBundled(dmLiveUpdateDDF::ResourceEntry* entries, uint32_t num_entries, dmResourceArchive::HArchiveIndexContainer archive_index)
{
    for(uint32_t i = 0; i < num_entries; ++i)
    {
        if (entries[i].m_Flags == dmLiveUpdateDDF::BUNDLED)
        {
            dmResourceArchive::Result res = dmResourceArchive::FindEntry(archive_index, entries[i].m_Hash.m_Data.m_Data, 0x0);
            if (res == dmResourceArchive::RESULT_NOT_FOUND)
            {
                // Manifest expect the resource to be bundled, but it is not in the archive index.
                dmLogError("Resource '%s' is expected to be in the bundle was not found. Resource was modified between publishing the bundle and publishing the manifest?", entries[i].m_Url);
                return RESULT_INVALID_DATA;
            }
        }
    }

    return RESULT_OK;
}

Result VerifyResourcesBundled(HFactory factory, Manifest* manifest)
{
    uint32_t entry_count = manifest->m_DDFData->m_Resources.m_Count;
    dmLiveUpdateDDF::ResourceEntry* entries = manifest->m_DDFData->m_Resources.m_Data;

    return VerifyResourcesBundled(entries, entry_count, factory->m_Manifest->m_ArchiveIndex);
}

static Result LoadFromManifest(const Manifest* manifest, const char* path, uint32_t* resource_size, LoadBufferType* buffer)
{
    dmhash_t path_hash = dmHashString64(path);

    int index = FindEntryIndex(manifest, path_hash);
    if (index < 0) {
        return RESULT_RESOURCE_NOT_FOUND; // Path not in manifest
    }

    dmLiveUpdateDDF::ResourceEntry* entries = manifest->m_DDFData->m_Resources.m_Data;
    dmResourceArchive::EntryData ed;
    dmResourceArchive::Result res = dmResourceArchive::FindEntry(manifest->m_ArchiveIndex, entries[index].m_Hash.m_Data.m_Data, &ed);
    if (res == dmResourceArchive::RESULT_OK)
    {
        uint32_t file_size = ed.m_ResourceSize;
        if (buffer->Capacity() < file_size)
        {
            buffer->SetCapacity(file_size);
        }

        buffer->SetSize(0);
        dmResourceArchive::Result read_result = dmResourceArchive::Read(manifest->m_ArchiveIndex, &ed, buffer->Begin());
        if (read_result != dmResourceArchive::RESULT_OK)
        {
            return RESULT_IO_ERROR;
        }

        buffer->SetSize(file_size);
        *resource_size = file_size;

        return RESULT_OK;
    }
    else if (res == dmResourceArchive::RESULT_NOT_FOUND)
    {
        // Resource was found in manifest, but not in archive
        return RESULT_RESOURCE_NOT_FOUND;
    }

    return RESULT_IO_ERROR;
}

// Assumes m_LoadMutex is already held
static Result DoLoadResourceLocked(HFactory factory, const char* path, const char* original_name, uint32_t* resource_size, LoadBufferType* buffer)
{
    DM_PROFILE(Resource, "LoadResource");
    if (factory->m_BuiltinsManifest)
    {
        if (LoadFromManifest(factory->m_BuiltinsManifest, original_name, resource_size, buffer) == RESULT_OK)
        {
            return RESULT_OK;
        }
    }

    char factory_path[RESOURCE_PATH_MAX];
    GetCanonicalPathFromBase(factory->m_UriParts.m_Path, path, factory_path);
    // NOTE: No else if here. Fall through
    if (factory->m_HttpClient)
    {
        // Load over HTTP
        *resource_size = 0;
        factory->m_HttpBuffer = buffer;
        factory->m_HttpContentLength = -1;
        factory->m_HttpTotalBytesStreamed = 0;
        factory->m_HttpFactoryResult = RESULT_OK;
        factory->m_HttpStatus = -1;

        char uri[RESOURCE_PATH_MAX*2];
        dmURI::Encode(factory_path, uri, sizeof(uri));

        dmHttpClient::Result http_result = dmHttpClient::Get(factory->m_HttpClient, uri);
        if (http_result != dmHttpClient::RESULT_OK)
        {
            if (factory->m_HttpStatus == 404)
            {
                return RESULT_RESOURCE_NOT_FOUND;
            }
            else
            {
                // 304 (NOT MODIFIED) is OK. 304 is returned when the resource is loaded from cache, ie ETag or similar match
                if (http_result == dmHttpClient::RESULT_NOT_200_OK && factory->m_HttpStatus != 304)
                {
                    dmLogWarning("Unexpected http status code: %d", factory->m_HttpStatus);
                    return RESULT_IO_ERROR;
                }
            }
        }

        if (factory->m_HttpFactoryResult != RESULT_OK)
            return factory->m_HttpFactoryResult;

        // Only check content-length if status != 304 (NOT MODIFIED)
        if (factory->m_HttpStatus != 304 && factory->m_HttpContentLength != -1 && factory->m_HttpContentLength != (int32_t)factory->m_HttpTotalBytesStreamed)
        {
            dmLogError("Expected content length differs from actually streamed for resource %s (%d != %d)", factory_path, factory->m_HttpContentLength, factory->m_HttpTotalBytesStreamed);
        }

        *resource_size = factory->m_HttpTotalBytesStreamed;
        return RESULT_OK;
    }
    else if (factory->m_Manifest)
    {
        Result r = LoadFromManifest(factory->m_Manifest, original_name, resource_size, buffer);
        return r;
    }
    else
    {
        const char* fs_path = factory_path;
        char fs_mount_path[RESOURCE_PATH_MAX];
        if (dmSys::RESULT_OK != dmSys::ResolveMountFileName(fs_mount_path, sizeof(fs_mount_path), factory_path))
        {
            // on some platforms, performing operations on non existing files will halt the engine
            // so we're better off returning here immediately
            return RESULT_RESOURCE_NOT_FOUND;
        }
        fs_path = fs_mount_path;

        // Load over local file system
        uint32_t file_size;
        dmSys::Result r = dmSys::ResourceSize(fs_path, &file_size);
        if (r != dmSys::RESULT_OK) {
            if (r == dmSys::RESULT_NOENT)
                return RESULT_RESOURCE_NOT_FOUND;
            else
                return RESULT_IO_ERROR;
        }

        if (buffer->Capacity() < file_size) {
            buffer->SetCapacity(file_size);
        }
        buffer->SetSize(0);

        r = dmSys::LoadResource(fs_path, buffer->Begin(), file_size, &file_size);
        if (r == dmSys::RESULT_OK) {
            buffer->SetSize(file_size);
            *resource_size = file_size;
            return RESULT_OK;
        } else {
            if (r == dmSys::RESULT_NOENT)
                return RESULT_RESOURCE_NOT_FOUND;
            else
                return RESULT_IO_ERROR;
        }
    }
}

// Takes the lock.
Result DoLoadResource(HFactory factory, const char* path, const char* original_name, uint32_t* resource_size, LoadBufferType* buffer)
{
    // Called from async queue so we wrap around a lock
    dmMutex::ScopedLock lk(factory->m_LoadMutex);
    return DoLoadResourceLocked(factory, path, original_name, resource_size, buffer);
}

// Assumes m_LoadMutex is already held
Result LoadResource(HFactory factory, const char* path, const char* original_name, void** buffer, uint32_t* resource_size)
{
    if (factory->m_Buffer.Capacity() != DEFAULT_BUFFER_SIZE) {
        factory->m_Buffer.SetCapacity(DEFAULT_BUFFER_SIZE);
    }
    factory->m_Buffer.SetSize(0);
    Result r = DoLoadResourceLocked(factory, path, original_name, resource_size, &factory->m_Buffer);
    if (r == RESULT_OK)
        *buffer = factory->m_Buffer.Begin();
    else
        *buffer = 0;
    return r;
}


static const char* GetExtFromPath(const char* name, char* buffer, uint32_t buffersize)
{
    const char* ext = strrchr(name, '.');
    if( !ext )
        return 0;

    int result = dmStrlCpy(buffer, ext, buffersize);
    if( result >= 0 )
    {
        return buffer;
    }
    return 0;
}

// Assumes m_LoadMutex is already held
static Result DoGet(HFactory factory, const char* name, void** resource)
{
    assert(name);
    assert(resource);

    DM_PROFILE(Resource, "Get");

    *resource = 0;

    char canonical_path[RESOURCE_PATH_MAX]; // The actual resource path. E.g. "/my/icon.texturec"
    GetCanonicalPath(name, canonical_path);

    uint64_t canonical_path_hash = dmHashBuffer64(canonical_path, strlen(canonical_path));

    // Try to get from already loaded resources
    SResourceDescriptor* rd = factory->m_Resources->Get(canonical_path_hash);
    if (rd)
    {
        assert(factory->m_ResourceToHash->Get((uintptr_t) rd->m_Resource));
        rd->m_ReferenceCount++;
        *resource = rd->m_Resource;
        return RESULT_OK;
    }

    if (factory->m_Resources->Full())
    {
        dmLogError("The max number of resources (%d) has been passed, tweak \"%s\" in the config file.", factory->m_Resources->Capacity(), MAX_RESOURCES_KEY);
        return RESULT_OUT_OF_RESOURCES;
    }

    char extbuffer[64];
    const char* ext = GetExtFromPath(canonical_path, extbuffer, sizeof(extbuffer));
    if (ext)
    {
        ext++;

        SResourceType* resource_type = FindResourceType(factory, ext);
        if (resource_type == 0)
        {
            dmLogError("Unknown resource type: %s", ext);
            return RESULT_UNKNOWN_RESOURCE_TYPE;
        }

        void *buffer;
        uint32_t file_size;
        Result result = LoadResource(factory, canonical_path, name, &buffer, &file_size);
        if (result != RESULT_OK) {
            if (result == RESULT_RESOURCE_NOT_FOUND) {
                dmLogWarning("Resource not found: %s", name);
            }
            return result;
        }

        assert(buffer == factory->m_Buffer.Begin());

        // TODO: We should *NOT* allocate SResource dynamically...
        SResourceDescriptor tmp_resource;
        memset(&tmp_resource, 0, sizeof(tmp_resource));
        tmp_resource.m_NameHash = canonical_path_hash;
        tmp_resource.m_ReferenceCount = 1;
        tmp_resource.m_ResourceType = (void*) resource_type;

        void *preload_data = 0;
        Result create_error = RESULT_OK;

        if (resource_type->m_PreloadFunction)
        {
            ResourcePreloadParams params;
            params.m_Factory = factory;
            params.m_Context = resource_type->m_Context;
            params.m_Buffer = buffer;
            params.m_BufferSize = file_size;
            params.m_PreloadData = &preload_data;
            params.m_Filename = name;
            params.m_HintInfo = 0; // No hinting now
            create_error = resource_type->m_PreloadFunction(params);
        }

        if (create_error == RESULT_OK)
        {
            tmp_resource.m_ResourceSizeOnDisc = file_size;
            tmp_resource.m_ResourceSize = 0; // Not everything will report a size (but instead rely on the disc size, sinze it's close enough)

            ResourceCreateParams params;
            params.m_Factory = factory;
            params.m_Context = resource_type->m_Context;
            params.m_Buffer = buffer;
            params.m_BufferSize = file_size;
            params.m_PreloadData = preload_data;
            params.m_Resource = &tmp_resource;
            params.m_Filename = name;
            create_error = resource_type->m_CreateFunction(params);
        }

        if (create_error == RESULT_OK && resource_type->m_PostCreateFunction)
        {
            ResourcePostCreateParams params;
            params.m_Factory = factory;
            params.m_Context = resource_type->m_Context;
            params.m_PreloadData = preload_data;
            params.m_Resource = &tmp_resource;
            for(;;)
            {
                create_error = resource_type->m_PostCreateFunction(params);
                if(create_error != RESULT_PENDING)
                    break;
                dmTime::Sleep(1000);
            }
        }

        // Restore to default buffer size
        factory->m_Buffer.SetSize(0);
        if (factory->m_Buffer.Capacity() != DEFAULT_BUFFER_SIZE) {
            factory->m_Buffer.SetCapacity(DEFAULT_BUFFER_SIZE);
        }

        if (create_error == RESULT_OK)
        {
            Result insert_error = InsertResource(factory, name, canonical_path_hash, &tmp_resource);
            if (insert_error == RESULT_OK)
            {
                *resource = tmp_resource.m_Resource;
                return RESULT_OK;
            }
            else
            {
                ResourceDestroyParams params;
                params.m_Factory = factory;
                params.m_Context = resource_type->m_Context;
                params.m_Resource = &tmp_resource;
                resource_type->m_DestroyFunction(params);
                return insert_error;
            }
        }
        else
        {
            dmLogWarning("Unable to create resource: %s", canonical_path);
            return create_error;
        }
    }
    else
    {
        dmLogWarning("Unable to load resource: '%s'. Missing file extension.", name);
        return RESULT_MISSING_FILE_EXTENSION;
    }
}

Result Get(HFactory factory, const char* name, void** resource)
{
    assert(name);
    assert(resource);
    *resource = 0;

    Result chk = CheckSuppliedResourcePath(name);
    if (chk != RESULT_OK)
        return chk;

    dmMutex::ScopedLock lk(factory->m_LoadMutex);

    dmArray<const char*>& stack = factory->m_GetResourceStack;
    if (factory->m_RecursionDepth == 0)
    {
        stack.SetSize(0);
    }

    ++factory->m_RecursionDepth;

    uint32_t n = stack.Size();
    for (uint32_t i=0;i<n;i++)
    {
        if (strcmp(stack[i], name) == 0)
        {
            dmLogError("Self referring resource detected");
            dmLogError("Reference chain:");
            for (uint32_t j = 0; j < n; ++j)
            {
                dmLogError("%d: %s", j, stack[j]);
            }
            dmLogError("%d: %s", n, name);
            --factory->m_RecursionDepth;
            return RESULT_RESOURCE_LOOP_ERROR;
        }
    }

    if (stack.Full())
    {
        stack.SetCapacity(stack.Capacity() + 16);
    }
    stack.Push(name);
    Result r = DoGet(factory, name, resource);
    stack.SetSize(stack.Size() - 1);
    --factory->m_RecursionDepth;
    return r;
}

SResourceDescriptor* FindByHash(HFactory factory, uint64_t canonical_path_hash)
{
    return factory->m_Resources->Get(canonical_path_hash);
}

Result InsertResource(HFactory factory, const char* path, uint64_t canonical_path_hash, SResourceDescriptor* descriptor)
{
    if (factory->m_Resources->Full())
    {
        dmLogError("The max number of resources (%d) has been passed, tweak \"%s\" in the config file.", factory->m_Resources->Capacity(), MAX_RESOURCES_KEY);
        return RESULT_OUT_OF_RESOURCES;
    }

    assert(descriptor->m_Resource);
    assert(descriptor->m_ReferenceCount == 1);

    factory->m_Resources->Put(canonical_path_hash, *descriptor);
    factory->m_ResourceToHash->Put((uintptr_t) descriptor->m_Resource, canonical_path_hash);
    if (factory->m_ResourceHashToFilename)
    {
        char canonical_path[RESOURCE_PATH_MAX];
        GetCanonicalPath(path, canonical_path);
        factory->m_ResourceHashToFilename->Put(canonical_path_hash, strdup(canonical_path));
    }

    return RESULT_OK;
}

Result GetRaw(HFactory factory, const char* name, void** resource, uint32_t* resource_size)
{
    DM_PROFILE(Resource, "GetRaw");

    assert(name);
    assert(resource);
    assert(resource_size);

    *resource = 0;
    *resource_size = 0;

    Result chk = CheckSuppliedResourcePath(name);
    if (chk != RESULT_OK)
        return chk;

    dmMutex::ScopedLock lk(factory->m_LoadMutex);

    char canonical_path[RESOURCE_PATH_MAX];
    GetCanonicalPath(name, canonical_path);

    void* buffer;
    uint32_t file_size;
    Result result = LoadResource(factory, canonical_path, name, &buffer, &file_size);
    if (result == RESULT_OK) {
        *resource = malloc(file_size);
        assert(buffer == factory->m_Buffer.Begin());
        memcpy(*resource, buffer, file_size);
        *resource_size = file_size;
    }
    return result;
}

static Result DoReloadResource(HFactory factory, const char* name, SResourceDescriptor** out_descriptor)
{
    char canonical_path[RESOURCE_PATH_MAX];
    GetCanonicalPath(name, canonical_path);

    uint64_t canonical_path_hash = dmHashBuffer64(canonical_path, strlen(canonical_path));

    SResourceDescriptor* rd = factory->m_Resources->Get(canonical_path_hash);

    if (out_descriptor)
        *out_descriptor = rd;

    if (rd == 0x0)
        return RESULT_RESOURCE_NOT_FOUND;

    SResourceType* resource_type = (SResourceType*) rd->m_ResourceType;
    if (!resource_type->m_RecreateFunction)
        return RESULT_NOT_SUPPORTED;

    void* buffer;
    uint32_t file_size;
    Result result = LoadResource(factory, canonical_path, name, &buffer, &file_size);
    if (result != RESULT_OK)
        return result;

    assert(buffer == factory->m_Buffer.Begin());

    ResourceRecreateParams params;
    params.m_Factory = factory;
    params.m_Context = resource_type->m_Context;
    params.m_Message = 0;
    params.m_Buffer = buffer;
    params.m_BufferSize = file_size;
    params.m_Resource = rd;
    params.m_Filename = name;
    rd->m_PrevResource = 0;
    Result create_result = resource_type->m_RecreateFunction(params);
    if (create_result == RESULT_OK)
    {
        params.m_Resource->m_ResourceSizeOnDisc = file_size;
        if (factory->m_ResourceReloadedCallbacks)
        {
            for (uint32_t i = 0; i < factory->m_ResourceReloadedCallbacks->Size(); ++i)
            {
                ResourceReloadedCallbackPair& pair = (*factory->m_ResourceReloadedCallbacks)[i];
                ResourceReloadedParams reload_params;
                reload_params.m_UserData = pair.m_UserData;
                reload_params.m_Resource = rd;
                reload_params.m_Name = name;
                pair.m_Callback(reload_params);
            }
        }
        if (rd->m_PrevResource) {
            SResourceDescriptor tmp_resource = *rd;
            tmp_resource.m_Resource = rd->m_PrevResource;
            ResourceDestroyParams params;
            params.m_Factory = factory;
            params.m_Context = resource_type->m_Context;
            params.m_Resource = &tmp_resource;
            dmResource::Result res = resource_type->m_DestroyFunction(params);
            rd->m_PrevResource = 0x0;
            return res;
        }
        return RESULT_OK;
    }
    else
    {
        return create_result;
    }
}

Result ReloadResource(HFactory factory, const char* name, SResourceDescriptor** out_descriptor)
{
    dmMutex::ScopedLock lk(factory->m_LoadMutex);

    // Always verify cache for reloaded resources
    if (factory->m_HttpCache)
        dmHttpCache::SetConsistencyPolicy(factory->m_HttpCache, dmHttpCache::CONSISTENCY_POLICY_VERIFY);

    Result result = DoReloadResource(factory, name, out_descriptor);

    switch (result)
    {
        case RESULT_OK:
            dmLogInfo("%s was successfully reloaded.", name);
            break;
        case RESULT_OUT_OF_MEMORY:
            dmLogError("Not enough memory to reload %s.", name);
            break;
        case RESULT_FORMAT_ERROR:
        case RESULT_CONSTANT_ERROR:
            dmLogError("%s has invalid format and could not be reloaded.", name);
            break;
        case RESULT_RESOURCE_NOT_FOUND:
            dmLogError("%s could not be reloaded since it was never loaded before.", name);
            break;
        case RESULT_NOT_SUPPORTED:
            dmLogWarning("Reloading of resource type %s not supported.", ((SResourceType*)(*out_descriptor)->m_ResourceType)->m_Extension);
            break;
        default:
            dmLogWarning("%s could not be reloaded, unknown error: %d.", name, result);
            break;
    }

    if (factory->m_HttpCache)
        dmHttpCache::SetConsistencyPolicy(factory->m_HttpCache, dmHttpCache::CONSISTENCY_POLICY_TRUST_CACHE);

    return result;
}

Result SetResource(HFactory factory, uint64_t hashed_name, void* data, uint32_t datasize)
{
    DM_PROFILE(Resource, "Set");

    dmMutex::ScopedLock lk(factory->m_LoadMutex);

    assert(data);

    SResourceDescriptor* rd = factory->m_Resources->Get(hashed_name);
    if (!rd) {
        return RESULT_RESOURCE_NOT_FOUND;
    }

    SResourceType* resource_type = (SResourceType*) rd->m_ResourceType;
    if (!resource_type->m_RecreateFunction)
        return RESULT_NOT_SUPPORTED;

    assert(data);
    assert(datasize > 0);

    ResourceRecreateParams params;
    params.m_Factory = factory;
    params.m_Context = resource_type->m_Context;
    params.m_Message = 0;
    params.m_Buffer = data;
    params.m_BufferSize = datasize;
    params.m_Resource = rd;
    params.m_Filename = 0;
    params.m_NameHash = hashed_name;
    Result create_result = resource_type->m_RecreateFunction(params);
    if (create_result == RESULT_OK)
    {
        if (factory->m_ResourceReloadedCallbacks)
        {
            for (uint32_t i = 0; i < factory->m_ResourceReloadedCallbacks->Size(); ++i)
            {
                ResourceReloadedCallbackPair& pair = (*factory->m_ResourceReloadedCallbacks)[i];
                ResourceReloadedParams params;
                params.m_UserData = pair.m_UserData;
                params.m_Resource = rd;
                params.m_Name = 0;
                params.m_NameHash = hashed_name;
                pair.m_Callback(params);
            }
        }
        return RESULT_OK;
    }
    else
    {
        return create_result;
    }

    return RESULT_OK;
}

Result SetResource(HFactory factory, uint64_t hashed_name, void* message)
{
    DM_PROFILE(Resource, "SetResource");

    dmMutex::ScopedLock lk(factory->m_LoadMutex);

    assert(message);

    SResourceDescriptor* rd = factory->m_Resources->Get(hashed_name);
    if (!rd) {
        return RESULT_RESOURCE_NOT_FOUND;
    }

    SResourceType* resource_type = (SResourceType*) rd->m_ResourceType;
    if (!resource_type->m_RecreateFunction)
        return RESULT_NOT_SUPPORTED;

    ResourceRecreateParams params;
    params.m_Factory = factory;
    params.m_Context = resource_type->m_Context;
    params.m_Message = message;
    params.m_Buffer = 0;
    params.m_BufferSize = 0;
    params.m_Resource = rd;
    params.m_Filename = 0;
    params.m_NameHash = hashed_name;
    Result create_result = resource_type->m_RecreateFunction(params);
    if (create_result == RESULT_OK)
    {
        if (factory->m_ResourceReloadedCallbacks)
        {
            for (uint32_t i = 0; i < factory->m_ResourceReloadedCallbacks->Size(); ++i)
            {
                ResourceReloadedCallbackPair& pair = (*factory->m_ResourceReloadedCallbacks)[i];
                ResourceReloadedParams params;
                params.m_UserData = pair.m_UserData;
                params.m_Resource = rd;
                params.m_Name = 0;
                params.m_NameHash = hashed_name;
                pair.m_Callback(params);
            }
        }
        return RESULT_OK;
    }
    else
    {
        return create_result;
    }

    return RESULT_OK;
}

Result GetType(HFactory factory, void* resource, ResourceType* type)
{
    assert(type);

    uint64_t* resource_hash = factory->m_ResourceToHash->Get((uintptr_t) resource);
    if (!resource_hash)
    {
        return RESULT_NOT_LOADED;
    }

    SResourceDescriptor* rd = factory->m_Resources->Get(*resource_hash);
    assert(rd);
    assert(rd->m_ReferenceCount > 0);
    *type = (ResourceType) rd->m_ResourceType;

    return RESULT_OK;
}

Result GetTypeFromExtension(HFactory factory, const char* extension, ResourceType* type)
{
    assert(type);

    SResourceType* resource_type = FindResourceType(factory, extension);
    if (resource_type)
    {
        *type = (ResourceType) resource_type;
        return RESULT_OK;
    }
    else
    {
        return RESULT_UNKNOWN_RESOURCE_TYPE;
    }
}

Result GetExtensionFromType(HFactory factory, ResourceType type, const char** extension)
{
    for (uint32_t i = 0; i < factory->m_ResourceTypesCount; ++i)
    {
        SResourceType* rt = &factory->m_ResourceTypes[i];

        if (((uintptr_t) rt) == type)
        {
            *extension = rt->m_Extension;
            return RESULT_OK;
        }
    }

    *extension = 0;
    return RESULT_UNKNOWN_RESOURCE_TYPE;
}

Result GetDescriptor(HFactory factory, const char* name, SResourceDescriptor* descriptor)
{
    char canonical_path[RESOURCE_PATH_MAX];
    GetCanonicalPath(name, canonical_path);

    uint64_t canonical_path_hash = dmHashBuffer64(canonical_path, strlen(canonical_path));

    SResourceDescriptor* tmp_descriptor = factory->m_Resources->Get(canonical_path_hash);
    if (tmp_descriptor)
    {
        *descriptor = *tmp_descriptor;
        return RESULT_OK;
    }
    else
    {
        return RESULT_NOT_LOADED;
    }
}

Result GetDescriptorWithExt(HFactory factory, uint64_t hashed_name, const uint64_t* exts, uint32_t ext_count, SResourceDescriptor* descriptor)
{
    SResourceDescriptor* tmp_descriptor = factory->m_Resources->Get(hashed_name);
    if (!tmp_descriptor) {
        return RESULT_NOT_LOADED;
    }

    SResourceType* type = (SResourceType*) tmp_descriptor->m_ResourceType;
    bool ext_match = ext_count == 0;
    if (!ext_match) {
        for (uint32_t i = 0; i < ext_count; ++i) {
            if (type->m_ExtensionHash == exts[i]) {
                ext_match = true;
                break;
            }
        }
    }
    if (ext_match) {
        *descriptor = *tmp_descriptor;
        return RESULT_OK;
    } else {
        return RESULT_INVALID_FILE_EXTENSION;
    }
}

void IncRef(HFactory factory, void* resource)
{
    uint64_t* resource_hash = factory->m_ResourceToHash->Get((uintptr_t) resource);
    assert(resource_hash);

    SResourceDescriptor* rd = factory->m_Resources->Get(*resource_hash);
    assert(rd);
    assert(rd->m_ReferenceCount > 0);
    ++rd->m_ReferenceCount;
}

// For unit testing
uint32_t GetRefCount(HFactory factory, void* resource)
{
    uint64_t* resource_hash = factory->m_ResourceToHash->Get((uintptr_t) resource);
    if(!resource_hash)
        return 0;
    SResourceDescriptor* rd = factory->m_Resources->Get(*resource_hash);
    assert(rd);
    return rd->m_ReferenceCount;
}

uint32_t GetRefCount(HFactory factory, dmhash_t identifier)
{
    SResourceDescriptor* rd = factory->m_Resources->Get(identifier);
    if(!rd)
        return 0;
    return rd->m_ReferenceCount;
}

void Release(HFactory factory, void* resource)
{
    DM_PROFILE(Resource, "Release");

    uint64_t* resource_hash = factory->m_ResourceToHash->Get((uintptr_t) resource);
    assert(resource_hash);

    SResourceDescriptor* rd = factory->m_Resources->Get(*resource_hash);
    assert(rd);
    assert(rd->m_ReferenceCount > 0);
    rd->m_ReferenceCount--;

    if (rd->m_ReferenceCount == 0)
    {
        SResourceType* resource_type = (SResourceType*) rd->m_ResourceType;

        DM_PROFILE_DYN(ResourceRelease, resource_type->m_Extension, resource_type->m_ExtensionHash);

        ResourceDestroyParams params;
        params.m_Factory = factory;
        params.m_Context = resource_type->m_Context;
        params.m_Resource = rd;
        resource_type->m_DestroyFunction(params);

        factory->m_ResourceToHash->Erase((uintptr_t) resource);
        factory->m_Resources->Erase(*resource_hash);
        if (factory->m_ResourceHashToFilename)
        {
            const char** s = factory->m_ResourceHashToFilename->Get(*resource_hash);
            factory->m_ResourceHashToFilename->Erase(*resource_hash);
            assert(s);
            free((void*) *s);
        }
    }
}

void RegisterResourceReloadedCallback(HFactory factory, ResourceReloadedCallback callback, void* user_data)
{
    if (factory->m_ResourceReloadedCallbacks)
    {
        if (factory->m_ResourceReloadedCallbacks->Full())
        {
            factory->m_ResourceReloadedCallbacks->SetCapacity(factory->m_ResourceReloadedCallbacks->Capacity() + 128);
        }
        ResourceReloadedCallbackPair pair;
        pair.m_Callback = callback;
        pair.m_UserData = user_data;
        factory->m_ResourceReloadedCallbacks->Push(pair);
    }
}

void UnregisterResourceReloadedCallback(HFactory factory, ResourceReloadedCallback callback, void* user_data)
{
    if (factory->m_ResourceReloadedCallbacks)
    {
        uint32_t i = 0;
        uint32_t size = factory->m_ResourceReloadedCallbacks->Size();
        while (i < size)
        {
            ResourceReloadedCallbackPair& pair = (*factory->m_ResourceReloadedCallbacks)[i];
            if (pair.m_Callback == callback && pair.m_UserData == user_data)
            {
                factory->m_ResourceReloadedCallbacks->EraseSwap(i);
                --size;
            }
            else
            {
                ++i;
            }
        }
    }
}

Result GetPath(HFactory factory, const void* resource, uint64_t* hash)
{
    uint64_t* resource_hash = factory->m_ResourceToHash->Get((uintptr_t)resource);
    if( resource_hash ) {
        *hash = *resource_hash;
        return RESULT_OK;
    }
    *hash = 0;
    return RESULT_RESOURCE_NOT_FOUND;
}

dmMutex::HMutex GetLoadMutex(const dmResource::HFactory factory)
{
    return factory->m_LoadMutex;
}

void ReleaseBuiltinsManifest(HFactory factory)
{
    if (factory->m_BuiltinsManifest)
    {
        dmResourceArchive::Delete(factory->m_BuiltinsManifest->m_ArchiveIndex);
        dmDDF::FreeMessage(factory->m_BuiltinsManifest->m_DDFData);
        dmDDF::FreeMessage(factory->m_BuiltinsManifest->m_DDF);
        factory->m_BuiltinsManifest->m_DDFData = 0;
        factory->m_BuiltinsManifest->m_DDF = 0;
        delete factory->m_BuiltinsManifest;
        factory->m_BuiltinsManifest = 0;
    }
}

struct ResourceIteratorCallbackInfo
{
    FResourceIterator   m_Callback;
    void*               m_Context;
    bool                m_ShouldContinue;
};

static void ResourceIteratorCallback(ResourceIteratorCallbackInfo* callback, const dmhash_t* id, SResourceDescriptor* resource)
{
    IteratorResource info;
    info.m_Id           = resource->m_NameHash;
    info.m_SizeOnDisc   = resource->m_ResourceSizeOnDisc;
    info.m_Size         = resource->m_ResourceSize ? resource->m_ResourceSize : resource->m_ResourceSizeOnDisc; // default to the size on disc if no in memory size was specified
    info.m_RefCount     = resource->m_ReferenceCount;

    if (callback->m_ShouldContinue)
    {
        callback->m_ShouldContinue = callback->m_Callback(info, callback->m_Context);
    }
}

void IterateResources(HFactory factory, FResourceIterator callback, void* user_ctx)
{
    DM_MUTEX_SCOPED_LOCK(factory->m_LoadMutex);
    ResourceIteratorCallbackInfo callback_info = {callback, user_ctx, true};
    factory->m_Resources->Iterate<>(&ResourceIteratorCallback, &callback_info);
}

const char* ResultToString(Result r)
{
    #define DM_RESOURCE_RESULT_TO_STRING_CASE(x) case RESULT_##x: return #x;
    switch (r)
    {
        DM_RESOURCE_RESULT_TO_STRING_CASE(OK);
        DM_RESOURCE_RESULT_TO_STRING_CASE(INVALID_DATA);
        DM_RESOURCE_RESULT_TO_STRING_CASE(DDF_ERROR);
        DM_RESOURCE_RESULT_TO_STRING_CASE(RESOURCE_NOT_FOUND);
        DM_RESOURCE_RESULT_TO_STRING_CASE(MISSING_FILE_EXTENSION);
        DM_RESOURCE_RESULT_TO_STRING_CASE(ALREADY_REGISTERED);
        DM_RESOURCE_RESULT_TO_STRING_CASE(INVAL);
        DM_RESOURCE_RESULT_TO_STRING_CASE(UNKNOWN_RESOURCE_TYPE);
        DM_RESOURCE_RESULT_TO_STRING_CASE(OUT_OF_MEMORY);
        DM_RESOURCE_RESULT_TO_STRING_CASE(IO_ERROR);
        DM_RESOURCE_RESULT_TO_STRING_CASE(NOT_LOADED);
        DM_RESOURCE_RESULT_TO_STRING_CASE(OUT_OF_RESOURCES);
        DM_RESOURCE_RESULT_TO_STRING_CASE(STREAMBUFFER_TOO_SMALL);
        DM_RESOURCE_RESULT_TO_STRING_CASE(FORMAT_ERROR);
        DM_RESOURCE_RESULT_TO_STRING_CASE(CONSTANT_ERROR);
        DM_RESOURCE_RESULT_TO_STRING_CASE(NOT_SUPPORTED);
        DM_RESOURCE_RESULT_TO_STRING_CASE(RESOURCE_LOOP_ERROR);
        DM_RESOURCE_RESULT_TO_STRING_CASE(PENDING);
        DM_RESOURCE_RESULT_TO_STRING_CASE(VERSION_MISMATCH);
        DM_RESOURCE_RESULT_TO_STRING_CASE(SIGNATURE_MISMATCH);
        DM_RESOURCE_RESULT_TO_STRING_CASE(UNKNOWN_ERROR);
        default: break;
    }
    #undef DM_RESOURCE_RESULT_TO_STRING_CASE
    return "RESULT_UNDEFINED";
}

}
