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

#ifndef RESOURCE_PRIVATE_H
#define RESOURCE_PRIVATE_H

#include <ddf/ddf.h>
#include "resource_archive.h"
#include "resource.h"

// Internal API that preloader needs to use.

namespace dmResource
{
    // This is both for the total resource path, ie m_UriParts.X concatenated with relative path
    const uint32_t RESOURCE_PATH_MAX = 1024;

    const uint32_t MAX_RESOURCE_TYPES = 128;

    struct SResourceType
    {
        SResourceType()
        {
            memset(this, 0, sizeof(*this));
        }
        dmhash_t            m_ExtensionHash;
        const char*         m_Extension;
        void*               m_Context;
        FResourcePreload    m_PreloadFunction;
        FResourceCreate     m_CreateFunction;
        FResourcePostCreate m_PostCreateFunction;
        FResourceDestroy    m_DestroyFunction;
        FResourceRecreate   m_RecreateFunction;
    };

    typedef dmArray<char> LoadBufferType;

    struct SResourceDescriptor;

    Result CheckSuppliedResourcePath(const char* name);

    // load with default internal buffer and its management, returns buffer ptr in 'buffer'
    Result LoadResource(HFactory factory, const char* path, const char* original_name, void** buffer, uint32_t* resource_size);
    // load with own buffer
    Result DoLoadResource(HFactory factory, const char* path, const char* original_name, uint32_t* resource_size, LoadBufferType* buffer);

    Result InsertResource(HFactory factory, const char* path, uint64_t canonical_path_hash, SResourceDescriptor* descriptor);
    uint32_t GetCanonicalPath(const char* relative_dir, char* buf);
    uint32_t GetCanonicalPathFromBase(const char* base_dir, const char* relative_dir, char* buf);

    SResourceType* FindResourceType(SResourceFactory* factory, const char* extension);
    uint32_t GetRefCount(HFactory factory, void* resource);
    uint32_t GetRefCount(HFactory factory, dmhash_t identifier);

    /**
     * The manifest has a signature embedded. This signature is created when bundling by hashing the manifest content
     * and encrypting the hash with the private part of a public-private key pair. To verify a manifest this procedure
     * is performed in reverse; first decrypting the signature using the public key (bundled with the engine) to
     * retreive the content hash then hashing the actual manifest content and comparing the two.
     * This method handles the signature decryption part.
     */
    Result DecryptSignatureHash(Manifest* manifest, const uint8_t* pub_key_buf, uint32_t pub_key_len, uint8_t** out_digest, uint32_t* out_digest_len);

    /**
     * In the case of an app-store upgrade, we dont want the runtime to load any existing local liveupdate.manifest.
     * We check this by persisting the bundled manifest signature to file the first time a liveupdate.manifest
     * is stored. At app start we check the current bundled manifest signature against the signature written to file.
     * If they don't match the bundle has changed, and we need to remove any liveupdate.manifest from the filesystem
     * and load the bundled manifest instead.
     */
    Result BundleVersionValid(const Manifest* manifest, const char* bundle_ver_path);

    /**
     * Exposed for unit tests
     */
     Result VerifyResourcesBundled(dmLiveUpdateDDF::ResourceEntry* entries, uint32_t num_entries, dmResourceArchive::HArchiveIndexContainer archive_index);

    struct PreloadRequest;
    struct PreloadHintInfo
    {
        HPreloader m_Preloader;
        int32_t m_Parent;
    };
}

#endif
