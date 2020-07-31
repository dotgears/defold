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

#include <stdint.h>
#include "../resource.h"
#include "../resource_private.h"
#include "../resource_archive.h"
#include "../resource_archive_private.h"

#if defined(__linux__) || defined(__MACH__) || defined(__EMSCRIPTEN__)
#include <netinet/in.h>
#elif defined(_WIN32)
#include <winsock2.h>
#else
#error "Unsupported platform"
#endif

#define JC_TEST_IMPLEMENTATION
#include <jc_test/jc_test.h>


template <> char* jc_test_print_value(char* buffer, size_t buffer_len, dmResource::Result r) {
    return buffer + JC_TEST_SNPRINTF(buffer, buffer_len, "%s", dmResource::ResultToString(r));
}

// new file format, generated test data
extern unsigned char RESOURCES_ARCI[];
extern uint32_t RESOURCES_ARCI_SIZE;
extern unsigned char RESOURCES_ARCD[];
extern uint32_t RESOURCES_ARCD_SIZE;
extern unsigned char RESOURCES_DMANIFEST[];
extern uint32_t RESOURCES_DMANIFEST_SIZE;
extern unsigned char RESOURCES_PUBLIC[];
extern uint32_t RESOURCES_PUBLIC_SIZE;
extern unsigned char RESOURCES_MANIFEST_HASH[];
extern uint32_t RESOURCES_MANIFEST_HASH_SIZE;

extern unsigned char RESOURCES_COMPRESSED_ARCI[];
extern uint32_t RESOURCES_COMPRESSED_ARCI_SIZE;
extern unsigned char RESOURCES_COMPRESSED_ARCD[];
extern uint32_t RESOURCES_COMPRESSED_ARCD_SIZE;
extern unsigned char RESOURCES_COMPRESSED_DMANIFEST[];
extern uint32_t RESOURCES_COMPRESSED_DMANIFEST_SIZE;

static const uint64_t path_hash[]       = { 0x1db7f0530911b1ce, 0x68b7e06402ee965c, 0x731d3cc48697dfe4, 0x8417331f14a42e4b,  0xb4870d43513879ba,  0xe1f97b41134ff4a6, 0xe7b921ca4d761083 };
static const char* path_name[]          = { "/archive_data/file4.adc", "/archive_data/liveupdate.file6.scriptc", "/archive_data/file5.scriptc", "/archive_data/file1.adc", "/archive_data/file3.adc",  "/archive_data/file2.adc", "/archive_data/liveupdate.file7.adc" };
static const char* content[]            = {
    "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    "this script was loaded sometime in runtime with liveupdate",
    "stuff to test encryption",
    "file1_datafile1_datafile1_data",
    "file3_data",
    "file2_datafile2_datafile2_data",
    "liveupdatefile1_datafile1_datafile1_data"
};

static const uint64_t liveupdate_path_hash[2] = { 0x68b7e06402ee965c, 0xe7b921ca4d761083 };

static const uint8_t sorted_first_hash[20] =
    {  0U,   1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U };
static const uint8_t sorted_middle_hash[20] =
    {  70U,  250U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U };
static const uint8_t sorted_last_hash[20] =
    { 226U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U };

static const uint8_t content_hash[][20] = {
    { 127U, 144U,   0U,  37U, 122U,  73U,  24U, 215U,   7U,  38U,  85U, 234U,  70U, 133U,  64U, 205U, 203U, 212U,  46U,  12U },
    { 205U,  82U, 220U, 208U,  16U, 146U, 230U, 113U, 118U,  43U,   6U,  77U,  19U,  47U, 181U, 219U, 201U,  63U,  81U, 143U },
    {  95U, 158U,  27U, 108U, 112U,  93U, 159U, 220U, 188U,  65U, 128U,  98U, 243U, 234U,  63U, 106U,  51U, 100U,   9U,  20U },
    { 225U, 251U, 249U, 131U,  22U, 226U, 178U, 216U, 248U, 181U, 222U, 168U, 119U, 247U,  11U,  53U, 176U,  14U,  43U, 170U },
    {   3U,  86U, 172U, 159U, 110U, 187U, 139U, 211U, 219U,   5U, 203U, 115U, 150U,  43U, 182U, 252U, 136U, 228U, 122U, 181U },
    {  69U,  26U,  15U, 239U, 138U, 110U, 167U, 120U, 214U,  38U, 144U, 200U,  19U, 102U,  63U,  48U, 173U,  41U,  21U,  66U },
    {  90U,  15U,  50U,  67U, 184U,   5U, 147U, 194U, 160U, 203U,  45U, 150U,  20U, 194U,  55U, 123U, 189U, 218U, 105U, 103U }
};
static const uint8_t compressed_content_hash[][20] = {
    { 206U, 246U, 241U, 188U, 170U, 142U,  34U, 244U, 115U,  87U,  65U,  38U,  88U,  34U, 188U,  33U, 144U,  44U,  18U,  46U },
    { 205U,  82U, 220U, 208U,  16U, 146U, 230U, 113U, 118U,  43U,   6U,  77U,  19U,  47U, 181U, 219U, 201U,  63U,  81U, 143U },
    {  95U, 158U,  27U, 108U, 112U,  93U, 159U, 220U, 188U,  65U, 128U,  98U, 243U, 234U,  63U, 106U,  51U, 100U,   9U,  20U },
    { 110U, 207U, 167U,  68U,  57U, 224U,  20U,  24U, 135U, 248U, 166U, 192U, 197U, 173U,  48U, 150U,   3U,  64U, 180U,  88U },
    {   3U,  86U, 172U, 159U, 110U, 187U, 139U, 211U, 219U,   5U, 203U, 115U, 150U,  43U, 182U, 252U, 136U, 228U, 122U, 181U },
    {  16U, 184U, 254U, 147U, 172U,  48U,  89U, 214U,  29U,  90U, 128U, 156U,  37U,  60U, 100U,  69U, 246U, 252U, 122U,  99U },
    {  90U,  15U,  50U,  67U, 184U,   5U, 147U, 194U, 160U, 203U,  45U, 150U,  20U, 194U,  55U, 123U, 189U, 218U, 105U, 103U }
};

static const uint32_t ENTRY_SIZE = sizeof(dmResourceArchive::EntryData) + DMRESOURCE_MAX_HASH;

void PopulateLiveUpdateResource(dmResourceArchive::LiveUpdateResource*& resource)
{
    uint32_t count = strlen(content[0]);
    resource->m_Data = (uint8_t*)content[0];
    resource->m_Count = count;
    resource->m_Header->m_Flags = 0;
    resource->m_Header->m_Size = 0;
}

void FreeLiveUpdateEntries(dmResourceArchive::LiveUpdateEntries*& liveupdate_entries)
{
    free(liveupdate_entries->m_Entries);
    free((void*)liveupdate_entries->m_Hashes);
    delete liveupdate_entries;
}

// Call to this should be free'd with FreeMutableIndexData(...)
void GetMutableIndexData(void*& arci_data, uint32_t num_entries_to_be_added)
{
    uint32_t index_alloc_size = RESOURCES_ARCI_SIZE + ENTRY_SIZE * num_entries_to_be_added;
    arci_data = malloc(index_alloc_size);
    memcpy(arci_data, RESOURCES_ARCI, RESOURCES_ARCI_SIZE);
}

// Call to this should be free'd with FreeMutableIndexData(...)
void GetMutableBundledIndexData(void*& arci_data, uint32_t& arci_size, uint32_t num_entries_to_keep)
{
    uint32_t num_lu_entries = 2 - num_entries_to_keep; // 2 LiveUpdate resources in archive in total
    ASSERT_EQ(true, num_lu_entries >= 0);
    arci_data = malloc(RESOURCES_ARCI_SIZE - ENTRY_SIZE * num_lu_entries);
    // Init archive container including LU resources
    dmResourceArchive::ArchiveIndexContainer* archive = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer(RESOURCES_ARCI, RESOURCES_ARCD, 0x0, 0x0, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

    uint32_t entry_count = htonl(archive->m_ArchiveIndex->m_EntryDataCount);
    uint32_t hash_offset = JAVA_TO_C(archive->m_ArchiveIndex->m_HashOffset);
    uint32_t entries_offset = JAVA_TO_C(archive->m_ArchiveIndex->m_EntryDataOffset);
    dmResourceArchive::EntryData* entries = (dmResourceArchive::EntryData*)((uintptr_t)archive->m_ArchiveIndex + entries_offset);

    // Construct "bundled" archive
    uint8_t* cursor = (uint8_t*)arci_data;
    uint8_t* cursor_hash = (uint8_t*)((uintptr_t)arci_data + hash_offset);
    uint8_t* cursor_entry = (uint8_t*)((uintptr_t)arci_data + entries_offset - num_lu_entries * DMRESOURCE_MAX_HASH);
    memcpy(cursor, RESOURCES_ARCI, sizeof(dmResourceArchive::ArchiveIndex)); // Copy header
    int lu_entries_to_copy = num_entries_to_keep;
    for (uint32_t i = 0; i < entry_count; ++i)
    {
        dmResourceArchive::EntryData& e = entries[i];
        bool is_lu_entry = JAVA_TO_C(e.m_Flags) & dmResourceArchive::ENTRY_FLAG_LIVEUPDATE_DATA;
        if (!is_lu_entry || lu_entries_to_copy > 0)
        {
            if (is_lu_entry)
            {
                --lu_entries_to_copy;
            }

            memcpy(cursor_hash, (void*)((uintptr_t)RESOURCES_ARCI + hash_offset + DMRESOURCE_MAX_HASH * i), DMRESOURCE_MAX_HASH);
            memcpy(cursor_entry, &e, sizeof(dmResourceArchive::EntryData));

            cursor_hash = (uint8_t*)((uintptr_t)cursor_hash + DMRESOURCE_MAX_HASH);
            cursor_entry = (uint8_t*)((uintptr_t)cursor_entry + sizeof(dmResourceArchive::EntryData));

        }
    }
    dmResourceArchive::ArchiveIndex* ai = (dmResourceArchive::ArchiveIndex*)arci_data;
    ai->m_EntryDataOffset = C_TO_JAVA(entries_offset - num_lu_entries * DMRESOURCE_MAX_HASH);
    ai->m_EntryDataCount = C_TO_JAVA(entry_count - num_lu_entries);

    arci_size = sizeof(dmResourceArchive::ArchiveIndex) + JAVA_TO_C(ai->m_EntryDataCount) * (ENTRY_SIZE);

    dmResourceArchive::Delete(archive);
}

void FreeMutableIndexData(void*& arci_data)
{
    free(arci_data);
}

bool IsLiveUpdateResource(dmhash_t lu_path_hash)
{
    for (uint32_t i = 0; i < (sizeof(liveupdate_path_hash) / sizeof(liveupdate_path_hash[0])); ++i)
    {
        if (lu_path_hash == liveupdate_path_hash[i])
        {
            return true;
        }
    }
    return false;
}

void CreateBundledArchive(dmResourceArchive::HArchiveIndexContainer& bundled_archive_container, dmResourceArchive::HArchiveIndex& bundled_archive_index, uint32_t num_entries_to_keep)
{
    bundled_archive_container = 0;
    bundled_archive_index = 0;
    uint32_t bundled_archive_size = 0;
    GetMutableBundledIndexData((void*&)bundled_archive_index, bundled_archive_size, num_entries_to_keep);
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*&) bundled_archive_index, RESOURCES_ARCD, 0x0, 0x0, 0x0, &bundled_archive_container);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(5U + num_entries_to_keep, dmResourceArchive::GetEntryCount(bundled_archive_container));
}

void FreeBundledArchive(dmResourceArchive::HArchiveIndexContainer& bundled_archive_container, dmResourceArchive::HArchiveIndex& bundled_archive_index)
{
    dmResourceArchive::Delete(bundled_archive_container);
    FreeMutableIndexData((void*&)bundled_archive_index);
}

TEST(dmResourceArchive, ShiftInsertResource)
{
    const char* resource_filename = "test_resource_liveupdate.arcd";
    FILE* resource_file = fopen(resource_filename, "wb");
    bool success = resource_file != 0x0;
    ASSERT_EQ(success, true);

    // Resource data to insert
    dmResourceArchive::LiveUpdateResource* resource = (dmResourceArchive::LiveUpdateResource*)malloc(sizeof(dmResourceArchive::LiveUpdateResource));
    resource->m_Header = (dmResourceArchive::LiveUpdateResourceHeader*)malloc(sizeof(dmResourceArchive::LiveUpdateResourceHeader));
    PopulateLiveUpdateResource(resource);

    // Use copy since we will shift/insert data
    uint8_t* arci_copy;
    GetMutableIndexData((void*&)arci_copy, 1);

    // Init archive container
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*) arci_copy, RESOURCES_ARCD, resource_filename, 0x0, resource_file, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    uint32_t entry_count_before = dmResourceArchive::GetEntryCount(archive);
    ASSERT_EQ(7U, entry_count_before);

    // Insertion
    int index = -1;
    dmResourceArchive::GetInsertionIndex(archive, sorted_middle_hash, &index);
    ASSERT_TRUE(index >= 0);
    dmResourceArchive::Result insert_result = dmResourceArchive::ShiftAndInsert(archive, 0x0, sorted_middle_hash, 20, index, resource, 0x0);
    ASSERT_EQ(insert_result, dmResourceArchive::RESULT_OK);
    uint32_t entry_count_after = dmResourceArchive::GetEntryCount(archive);
    ASSERT_EQ(8U, entry_count_after);

    // Find inserted entry in archive after insertion
    dmResourceArchive::EntryData entry;
    result = dmResourceArchive::FindEntry(archive, sorted_middle_hash, &entry);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(entry.m_ResourceSize, resource->m_Count);

    free(resource->m_Header);
    free(resource);
    dmResourceArchive::Delete(archive);
    FreeMutableIndexData((void*&)arci_copy);
    remove(resource_filename);
}

TEST(dmResourceArchive, NewArchiveIndexFromCopy)
{
    uint32_t single_entry_offset = DMRESOURCE_MAX_HASH;

    dmResourceArchive::HArchiveIndexContainer archive_container = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*) RESOURCES_ARCI, RESOURCES_ARCD, 0x0, 0x0, 0x0, &archive_container);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(496U, dmResourceArchive::GetEntryDataOffset(archive_container));

    // No extra allocation
    dmResourceArchive::HArchiveIndex dst_archive = 0;
    dmResourceArchive::NewArchiveIndexFromCopy(dst_archive, archive_container, 0);
    ASSERT_EQ(496U, dmResourceArchive::GetEntryDataOffset(dst_archive));
    dmResourceArchive::Delete(dst_archive);

    // Allocate space for 3 extra entries
    dst_archive = 0;
    dmResourceArchive::NewArchiveIndexFromCopy(dst_archive, archive_container, 3);
    ASSERT_EQ(496U + 3 * single_entry_offset, dmResourceArchive::GetEntryDataOffset(dst_archive));
    dmResourceArchive::Delete(dst_archive);

    dmResourceArchive::Delete(archive_container);
}

TEST(dmResourceArchive, CacheLiveUpdateEntries)
{
    dmResourceArchive::HArchiveIndexContainer bundled_archive_container;
    dmResourceArchive::ArchiveIndex* bundled_archive_index;

    dmResourceArchive::HArchiveIndexContainer archive_container = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*) RESOURCES_ARCI, RESOURCES_ARCD, 0x0, 0x0, 0x0, &archive_container);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(7U, dmResourceArchive::GetEntryCount(archive_container));

    // Create "bundled" archive container that does not contain any LiveUpdate resources
    CreateBundledArchive(bundled_archive_container, bundled_archive_index, 0);
    dmResourceArchive::LiveUpdateEntries* liveupdate_entries = new dmResourceArchive::LiveUpdateEntries;
    dmResourceArchive::CacheLiveUpdateEntries(archive_container, bundled_archive_container, liveupdate_entries);
    ASSERT_EQ(2U, liveupdate_entries->m_Count);
    FreeLiveUpdateEntries(liveupdate_entries);
    FreeBundledArchive(bundled_archive_container, bundled_archive_index);

    // Create "bundled" archive container that has 1 LiveUpdate entry now in bundle instead
    CreateBundledArchive(bundled_archive_container, bundled_archive_index, 1);
    liveupdate_entries = new dmResourceArchive::LiveUpdateEntries;
    dmResourceArchive::CacheLiveUpdateEntries(archive_container, bundled_archive_container, liveupdate_entries);
    ASSERT_EQ(1U, liveupdate_entries->m_Count);
    FreeLiveUpdateEntries(liveupdate_entries);
    FreeBundledArchive(bundled_archive_container, bundled_archive_index);

    dmResourceArchive::Delete(archive_container);
}

TEST(dmResourceArchive, GetInsertionIndex)
{
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*) RESOURCES_ARCI, RESOURCES_ARCD, 0x0, 0x0, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(7U, dmResourceArchive::GetEntryCount(archive));

    int index = -1;

    dmResourceArchive::GetInsertionIndex(archive, sorted_first_hash, &index);
    ASSERT_EQ(0, index);

    dmResourceArchive::GetInsertionIndex(archive, sorted_middle_hash, &index);
    ASSERT_EQ(2, index);

    dmResourceArchive::GetInsertionIndex(archive, sorted_last_hash, &index);
    ASSERT_EQ(7, index);

    dmResourceArchive::Delete(archive);
}

TEST(dmResourceArchive, ManifestHeader)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    dmLiveUpdateDDF::ManifestData* manifest_data;
    dmResource::Result result = dmResource::ManifestLoadMessage(RESOURCES_DMANIFEST, RESOURCES_DMANIFEST_SIZE, manifest);
    ASSERT_EQ(dmResource::RESULT_OK, result);

    manifest_data = manifest->m_DDFData;

    ASSERT_EQ(dmResource::MANIFEST_MAGIC_NUMBER, manifest_data->m_Header.m_MagicNumber);
    ASSERT_EQ(dmResource::MANIFEST_VERSION, manifest_data->m_Header.m_Version);

    ASSERT_EQ(dmLiveUpdateDDF::HASH_SHA1, manifest_data->m_Header.m_ResourceHashAlgorithm);
    ASSERT_EQ(dmLiveUpdateDDF::HASH_SHA256, manifest_data->m_Header.m_SignatureHashAlgorithm);

    ASSERT_EQ(dmLiveUpdateDDF::SIGN_RSA, manifest_data->m_Header.m_SignatureSignAlgorithm);

    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}

static void PrintHash(const uint8_t* hash, uint32_t len)
{
    char* buf = new char[len*2+1];
    buf[len] = '\0';
    for (uint32_t i = 0; i < len; ++i)
    {
        sprintf(buf+i*2, "%02X", hash[i]);
    }
    printf("HASH: %s\n", buf);
    delete[] buf;
}

static void PrintArray(uint8_t* a, uint32_t size)
{
    uint32_t left = size;
    uint32_t stride = 32;
    while (left > 0) {
        if (left > 8) {
            for (int i = 0; i < 8; ++i, ++a) {
                printf("%02X ", *a);
            }
            left -= 8;
            printf(" ");
        } else {
            for (int i = 0; i < left; ++i, ++a) {
                printf("%02X ", *a);
            }
            left  = 0;
        }
        stride -= 8;
        if (stride == 0) {
            printf("\n");
            stride = 32;
        }
    }
}

/*
This test is failing intermittenly on Linux. Typical output from a failed test:

2020-04-19T09:57:44.1778093Z ManifestSignatureVerification
2020-04-19T09:57:44.1779593Z PUBLIC KEY (sz: 162):
2020-04-19T09:57:44.1780587Z
2020-04-19T09:57:44.1782352Z 30 81 9F 30 0D 06 09 2A  86 48 86 F7 0D 01 01 01  05 00 03 81 8D 00 30 81  89 02 81 81 00 A4 6F BC
2020-04-19T09:57:44.1784180Z 37 EF BA F9 60 0D 98 02  AE 97 44 B2 52 C0 0F 0E  73 95 7B 97 B0 B7 08 6D  F2 D7 4E C3 37 92 7C BF
2020-04-19T09:57:44.1785946Z 55 DD 3A BD 8D 4D 73 94  3F 1B 69 70 86 23 DE 07  6D FF 46 86 17 6E 64 5B  8C 1F 36 AA 16 65 87 91
2020-04-19T09:57:44.1787741Z 53 F4 1C 9F AC C9 31 0C  93 BA E8 CB BB 5A AE F9  FD DC 2F C0 10 5B 6A 1B  0B DE 22 50 B3 E6 D9 AD
2020-04-19T09:57:44.1789537Z A7 E2 1C 92 D1 69 CA 93  25 AC 98 28 02 EF 1F F6  BF 67 7F 59 FB 1B 54 B0  29 DC 24 91 8D 02 03 01
2020-04-19T09:57:44.1790849Z 00 01
2020-04-19T09:57:44.1791703Z
2020-04-19T09:57:44.1792941Z MANIFEST SIGNATURE (sz: 128):
2020-04-19T09:57:44.1793804Z
2020-04-19T09:57:44.1795548Z 32 7B 7A DE B2 CE CB A1  A6 B1 34 1D EF F7 51 36  8D 7A A1 B1 90 49 5A 2E  A9 D7 FB 69 05 6A 5D B9
2020-04-19T09:57:44.1797361Z DC 94 91 EC E9 15 00 4E  51 49 C8 99 60 A6 F0 5F  DC 2B 8C 27 FB C8 70 5A  4C A5 40 9C D7 7E DD 95
2020-04-19T09:57:44.1799205Z 28 D1 8F 37 5E DA 60 16  52 84 12 BA E6 74 5D 09  ED 43 43 DD FA 47 91 D1  9A DC AA DB C7 CC DE B1
2020-04-19T09:57:44.1802608Z DF 30 C9 BA 2A 0A A8 A4  F1 4F 9D 8C AB 68 80 98  D2 0E F9 AB 00 80 B2 9C  21 09 D2 36 37 25 16 AF
2020-04-19T09:57:44.1805102Z
2020-04-19T09:57:44.1807577Z
2020-04-19T09:57:44.1810518Z ../src/test/test_resource_archive.cpp:392:
2020-04-19T09:57:44.1814584Z Expected: (dmResource::RESULT_OK) == (dmResource::DecryptSignatureHash(manifest, RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len)), actual: OK vs INVALID_DATA
*/
#if !defined(__linux__)
TEST(dmResourceArchive, ManifestSignatureVerification)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::ManifestLoadMessage(RESOURCES_DMANIFEST, RESOURCES_DMANIFEST_SIZE, manifest));

    uint32_t expected_digest_len = dmResource::HashLength(manifest->m_DDFData->m_Header.m_SignatureHashAlgorithm);
    uint8_t* expected_digest = (uint8_t*)RESOURCES_MANIFEST_HASH;

    // We have an intermittent fail here, so let's output the info so we can start investigating it.
    // We always print these so that we can compare the failed build with a successful one
    {
        printf("\nPUBLIC KEY (sz: %u):\n\n", RESOURCES_PUBLIC_SIZE);
        PrintArray(RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE);
        printf("\n");

        uint8_t* signature = manifest->m_DDF->m_Signature.m_Data;
        uint32_t signature_len = manifest->m_DDF->m_Signature.m_Count;
        printf("\nMANIFEST SIGNATURE (sz: %u):\n\n", signature_len);
        PrintArray(signature, signature_len);
        printf("\n");
    }
    uint8_t* hex_digest = 0x0;
    uint32_t hex_digest_len;
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::DecryptSignatureHash(manifest, RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len));

    // debug prints to determine cause of intermittent test fail on linux 32-bit
    printf("Expected digest (%u bytes):\n", expected_digest_len);
    PrintHash((const uint8_t*)expected_digest, expected_digest_len);
    printf("Actual digest (%u bytes):\n", hex_digest_len);
    PrintHash((const uint8_t*)hex_digest, hex_digest_len);
    // end debug

    ASSERT_EQ(dmResource::RESULT_OK, dmResource::HashCompare((const uint8_t*) hex_digest, hex_digest_len, (const uint8_t*) expected_digest, expected_digest_len));

    free(hex_digest);
    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}
#endif

/*
This test is failing intermittenly on Linux. Typical output from a failed test:

2020-04-24T11:09:51.7615960Z ManifestSignatureVerificationLengthFail
2020-04-24T11:09:51.7616210Z ../src/test/test_resource_archive.cpp:445:
2020-04-24T11:09:51.7616493Z Expected: (dmResource::RESULT_OK) == (dmResource::DecryptSignatureHash(manifest, RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len)), actual: OK vs INVALID_DATA
2020-04-24T11:09:51.7616663Z
*/
#if !defined(__linux__)
TEST(dmResourceArchive, ManifestSignatureVerificationLengthFail)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::ManifestLoadMessage(RESOURCES_DMANIFEST, RESOURCES_DMANIFEST_SIZE, manifest));

    uint32_t expected_digest_len = dmResource::HashLength(manifest->m_DDFData->m_Header.m_SignatureHashAlgorithm);
    uint8_t* expected_digest = (uint8_t*)RESOURCES_MANIFEST_HASH;

    uint8_t* hex_digest = 0x0;
    uint32_t hex_digest_len;
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::DecryptSignatureHash(manifest, RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len));
    hex_digest_len *= 0.5f; // make the supplied hash shorter than expected
    ASSERT_EQ(dmResource::RESULT_FORMAT_ERROR, dmResource::HashCompare(hex_digest, hex_digest_len, expected_digest, expected_digest_len));

    free(hex_digest);
    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}
#endif

/*
This test is failing intermittenly on Linux. Typical output from a failed test:

2020-04-28T05:00:04.1089407Z ManifestSignatureVerificationHashFail
2020-04-28T05:00:04.1089610Z ../src/test/test_resource_archive.cpp:475:
2020-04-28T05:00:04.1089868Z Expected: (dmResource::RESULT_OK) == (dmResource::DecryptSignatureHash(manifest, RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len)), actual: OK vs INVALID_DATA
*/
#if !defined(__linux__)
TEST(dmResourceArchive, ManifestSignatureVerificationHashFail)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::ManifestLoadMessage(RESOURCES_DMANIFEST, RESOURCES_DMANIFEST_SIZE, manifest));

    uint32_t expected_digest_len = dmResource::HashLength(manifest->m_DDFData->m_Header.m_SignatureHashAlgorithm);
    uint8_t* expected_digest = (uint8_t*)RESOURCES_MANIFEST_HASH;

    uint8_t* hex_digest = 0x0;
    uint32_t hex_digest_len;
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::DecryptSignatureHash(manifest, RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len));
    memset(hex_digest, 0x0, hex_digest_len / 2); // NULL out the first half of hash
    ASSERT_EQ(dmResource::RESULT_FORMAT_ERROR, dmResource::HashCompare(hex_digest, hex_digest_len, expected_digest, expected_digest_len));

    free(hex_digest);
    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}
#endif

TEST(dmResourceArchive, ManifestSignatureVerificationWrongKey)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    ASSERT_EQ(dmResource::RESULT_OK, dmResource::ManifestLoadMessage(RESOURCES_DMANIFEST, RESOURCES_DMANIFEST_SIZE, manifest));

    unsigned char* resources_public_wrong = (unsigned char*)malloc(RESOURCES_PUBLIC_SIZE);
    memcpy(resources_public_wrong, &RESOURCES_PUBLIC, RESOURCES_PUBLIC_SIZE);
    resources_public_wrong[0] = RESOURCES_PUBLIC[0] + 1; // make the key invalid
    uint8_t* hex_digest = 0x0;
    uint32_t hex_digest_len;
    ASSERT_EQ(dmResource::RESULT_INVALID_DATA, dmResource::DecryptSignatureHash(manifest, resources_public_wrong, RESOURCES_PUBLIC_SIZE, &hex_digest, &hex_digest_len));

    free(hex_digest);
    free(resources_public_wrong);
    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}

TEST(dmResourceArchive, ResourceEntries)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    dmLiveUpdateDDF::ManifestData* manifest_data;
    dmResource::Result result = dmResource::ManifestLoadMessage(RESOURCES_DMANIFEST, RESOURCES_DMANIFEST_SIZE, manifest);
    ASSERT_EQ(dmResource::RESULT_OK, result);

    manifest_data = manifest->m_DDFData;

    ASSERT_EQ(7U, manifest_data->m_Resources.m_Count);
    for (uint32_t i = 0; i < manifest_data->m_Resources.m_Count; ++i) {
        const char* current_path = manifest_data->m_Resources.m_Data[i].m_Url;
        uint64_t current_hash = dmHashString64(current_path);

        if (IsLiveUpdateResource(current_hash)) continue;

        ASSERT_STREQ(path_name[i], current_path);
        ASSERT_EQ(path_hash[i], current_hash);

        for (uint32_t n = 0; n < manifest_data->m_Resources.m_Data[i].m_Hash.m_Data.m_Count; ++n) {
            uint8_t current_byte = manifest_data->m_Resources.m_Data[i].m_Hash.m_Data.m_Data[n];
            ASSERT_EQ(content_hash[i][n], current_byte);
        }
    }

    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}

TEST(dmResourceArchive, ResourceEntries_Compressed)
{
    dmResource::Manifest* manifest = new dmResource::Manifest();
    dmLiveUpdateDDF::ManifestData* manifest_data;
    dmResource::Result result = dmResource::ManifestLoadMessage(RESOURCES_COMPRESSED_DMANIFEST, RESOURCES_COMPRESSED_DMANIFEST_SIZE, manifest);
    ASSERT_EQ(dmResource::RESULT_OK, result);

    manifest_data = manifest->m_DDFData;

    ASSERT_EQ(7U, manifest_data->m_Resources.m_Count);
    for (uint32_t i = 0; i < manifest_data->m_Resources.m_Count; ++i) {
        const char* current_path = manifest_data->m_Resources.m_Data[i].m_Url;
        uint64_t current_hash = dmHashString64(current_path);

        if (IsLiveUpdateResource(current_hash)) continue;

        ASSERT_STREQ(path_name[i], current_path);
        ASSERT_EQ(path_hash[i], current_hash);

        for (uint32_t n = 0; n < manifest_data->m_Resources.m_Data[i].m_Hash.m_Data.m_Count; ++n) {
            uint8_t current_byte = manifest_data->m_Resources.m_Data[i].m_Hash.m_Data.m_Data[n];

            ASSERT_EQ(compressed_content_hash[i][n], current_byte);
        }
    }

    dmDDF::FreeMessage(manifest->m_DDFData);
    dmDDF::FreeMessage(manifest->m_DDF);
    delete manifest;
}

TEST(dmResourceArchive, Wrap)
{
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*) RESOURCES_ARCI, RESOURCES_ARCD, 0x0, 0x0, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(7U, dmResourceArchive::GetEntryCount(archive));

    dmResourceArchive::EntryData entry;
    for (uint32_t i = 0; i < (sizeof(path_hash) / sizeof(path_hash[0])); ++i)
    {
        if (IsLiveUpdateResource(path_hash[i])) continue;

        char buffer[1024] = { 0 };
        result = dmResourceArchive::FindEntry(archive, content_hash[i], &entry);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        result = dmResourceArchive::Read(archive, &entry, buffer);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        ASSERT_EQ(strlen(content[i]), strlen(buffer));
        ASSERT_STREQ(content[i], buffer);
    }

    uint8_t invalid_hash[] = { 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U };
    result = dmResourceArchive::FindEntry(archive, invalid_hash, &entry);
    ASSERT_EQ(dmResourceArchive::RESULT_NOT_FOUND, result);

    dmResourceArchive::Delete(archive);
}

TEST(dmResourceArchive, Wrap_Compressed)
{
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    dmResourceArchive::Result result = dmResourceArchive::WrapArchiveBuffer((void*) RESOURCES_COMPRESSED_ARCI, (void*) RESOURCES_COMPRESSED_ARCD, 0x0, 0x0, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(7U, dmResourceArchive::GetEntryCount(archive));

    dmResourceArchive::EntryData entry;
    for (uint32_t i = 0; i < (sizeof(path_hash) / sizeof(path_hash[0])); ++i)
    {
        if (IsLiveUpdateResource(path_hash[i])) continue;

        char buffer[1024] = { 0 };
        result = dmResourceArchive::FindEntry(archive, compressed_content_hash[i], &entry);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        result = dmResourceArchive::Read(archive, &entry, buffer);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        ASSERT_EQ(strlen(content[i]), strlen(buffer));
        ASSERT_STREQ(content[i], buffer);
    }

    uint8_t invalid_hash[] = { 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U };
    result = dmResourceArchive::FindEntry(archive, invalid_hash, &entry);
    ASSERT_EQ(dmResourceArchive::RESULT_NOT_FOUND, result);

    dmResourceArchive::Delete(archive);
}

TEST(dmResourceArchive, LoadFromDisk)
{
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    const char* archive_path = "build/default/src/test/resources.arci";
    const char* resource_path = "build/default/src/test/resources.arcd";
    dmResourceArchive::Result result = dmResourceArchive::LoadArchive(archive_path, resource_path, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(7U, dmResourceArchive::GetEntryCount(archive));

    dmResourceArchive::EntryData entry;
    for (uint32_t i = 0; i < sizeof(path_name)/sizeof(path_name[0]); ++i)
    {
        if (IsLiveUpdateResource(path_hash[i])) continue;

        char buffer[1024] = { 0 };
        result = dmResourceArchive::FindEntry(archive, content_hash[i], &entry);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        result = dmResourceArchive::Read(archive, &entry, buffer);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        ASSERT_EQ(strlen(content[i]), strlen(buffer));
        ASSERT_STREQ(content[i], buffer);
    }

    uint8_t invalid_hash[] = { 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U };
    result = dmResourceArchive::FindEntry(archive, invalid_hash, &entry);
    ASSERT_EQ(dmResourceArchive::RESULT_NOT_FOUND, result);

    dmResourceArchive::Delete(archive);
}

TEST(dmResourceArchive, LoadFromDisk_MissingArchive)
{
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    const char* archive_path = "build/default/src/test/missing-archive.arci";
    const char* resource_path = "build/default/src/test/resources.arcd";
    dmResourceArchive::Result result = dmResourceArchive::LoadArchive(archive_path, resource_path, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_IO_ERROR, result);
}

TEST(dmResourceArchive, LoadFromDisk_Compressed)
{
    dmResourceArchive::HArchiveIndexContainer archive = 0;
    const char* archive_path = "build/default/src/test/resources_compressed.arci";
    const char* resource_path = "build/default/src/test/resources_compressed.arcd";
    dmResourceArchive::Result result = dmResourceArchive::LoadArchive(archive_path, resource_path, 0x0, &archive);
    ASSERT_EQ(dmResourceArchive::RESULT_OK, result);
    ASSERT_EQ(7U, dmResourceArchive::GetEntryCount(archive));

    dmResourceArchive::EntryData entry;
    for (uint32_t i = 0; i < sizeof(path_name)/sizeof(path_name[0]); ++i)
    {
        if (IsLiveUpdateResource(path_hash[i])) continue;

        char buffer[1024] = { 0 };
        result = dmResourceArchive::FindEntry(archive, compressed_content_hash[i], &entry);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        result = dmResourceArchive::Read(archive, &entry, buffer);
        ASSERT_EQ(dmResourceArchive::RESULT_OK, result);

        ASSERT_EQ(strlen(content[i]), strlen(buffer));
        ASSERT_STREQ(content[i], buffer);
    }

    uint8_t invalid_hash[] = { 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U };
    result = dmResourceArchive::FindEntry(archive, invalid_hash, &entry);
    ASSERT_EQ(dmResourceArchive::RESULT_NOT_FOUND, result);

    dmResourceArchive::Delete(archive);
}

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);
    int ret = jc_test_run_all();
    return ret;
}
