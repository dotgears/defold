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

#ifndef RESOURCE_H
#define RESOURCE_H

#include <ddf/ddf.h>
#include <dlib/array.h>
#include <dlib/hash.h>
#include <dlib/mutex.h>
#include "liveupdate_ddf.h"
#include "resource_archive.h"

namespace dmResource
{
    const static uint32_t MANIFEST_MAGIC_NUMBER = 0x43cb6d06;

    const static uint32_t MANIFEST_VERSION = 0x03;

    const uint32_t MANIFEST_PROJ_ID_LEN = 41; // SHA1 + NULL terminator

    /**
     * Configuration key used to tweak the max number of resources allowed.
     */
    extern const char* MAX_RESOURCES_KEY;

    /**
     * Empty flags
     */
    #define RESOURCE_FACTORY_FLAGS_EMPTY            (0)

    /**
     * Enable resource reloading support. Both over files and http.
     */
    #define RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT (1 << 0)

    /**
     * Enable HTTP cache
     */
    #define RESOURCE_FACTORY_FLAGS_HTTP_CACHE     (1 << 2)

    /**
     * Result
     */
    enum Result
    {
        RESULT_OK                        = 0,    //!< RESULT_OK
        RESULT_INVALID_DATA              = -1,   //!< RESULT_INVALID_DATA
        RESULT_DDF_ERROR                 = -2,   //!< RESULT_DDF_ERROR
        RESULT_RESOURCE_NOT_FOUND        = -3,   //!< RESULT_RESOURCE_NOT_FOUND
        RESULT_MISSING_FILE_EXTENSION    = -4,   //!< RESULT_MISSING_FILE_EXTENSION
        RESULT_ALREADY_REGISTERED        = -5,   //!< RESULT_ALREADY_REGISTERED
        RESULT_INVAL                     = -6,   //!< RESULT_INVAL
        RESULT_UNKNOWN_RESOURCE_TYPE     = -7,   //!< RESULT_UNKNOWN_RESOURCE_TYPE
        RESULT_OUT_OF_MEMORY             = -8,   //!< RESULT_OUT_OF_MEMORY
        RESULT_IO_ERROR                  = -9,   //!< RESULT_IO_ERROR
        RESULT_NOT_LOADED                = -10,  //!< RESULT_NOT_LOADED
        RESULT_OUT_OF_RESOURCES          = -11,  //!< RESULT_OUT_OF_RESOURCES
        RESULT_STREAMBUFFER_TOO_SMALL    = -12,  //!< RESULT_STREAMBUFFER_TOO_SMALL
        RESULT_FORMAT_ERROR              = -13,  //!< RESULT_FORMAT_ERROR
        RESULT_CONSTANT_ERROR            = -14,  //!< RESULT_CONSTANT_ERROR
        RESULT_NOT_SUPPORTED             = -15,  //!< RESULT_NOT_SUPPORTED
        RESULT_RESOURCE_LOOP_ERROR       = -16,  //!< RESULT_RESOURCE_LOOP_ERROR
        RESULT_PENDING                   = -17,  //!< RESULT_PENDING
        RESULT_INVALID_FILE_EXTENSION    = -18,  //!< RESULT_INVALID_FILE_EXTENSION
        RESULT_VERSION_MISMATCH          = -19,  //!< RESULT_VERSION_MISMATCH
        RESULT_SIGNATURE_MISMATCH        = -20,  //!< RESULT_SIGNATURE_MISMATCH
        RESULT_UNKNOWN_ERROR             = -21,  //!< RESULT_UNKNOWN_ERROR
    };

    /**
     * Resource kind
     */
    enum Kind
    {
        KIND_DDF_DATA,//!< KIND_DDF_DATA
        KIND_POINTER, //!< KIND_POINTER
    };

    struct Manifest
    {
        Manifest()
        {
            memset(this, 0, sizeof(Manifest));
        }

        dmResourceArchive::HArchiveIndexContainer   m_ArchiveIndex;
        dmLiveUpdateDDF::ManifestFile*              m_DDF;
        dmLiveUpdateDDF::ManifestData*              m_DDFData;
    };

    /// Resource descriptor
    struct SResourceDescriptor
    {
        /// Hash of resource name
        uint64_t m_NameHash;

        /// Resource pointer. Must be unique and not NULL.
        void*    m_Resource;
        /// Resource pointer to a previous version of the resource, iff it exists. Only used when recreating resources.
        void*    m_PrevResource;

        /// Resource size in memory. The payload of m_Resource
        uint32_t m_ResourceSize;

        /// Resource size on disc
        uint32_t m_ResourceSizeOnDisc;

        /// For internal use only
        void*    m_ResourceType;        // For internal use.

        /// Reference count
        uint32_t m_ReferenceCount;

        /// Resource kind
        Kind     m_ResourceKind;
    };

    /**
     * Factory handle
     */
    typedef struct SResourceFactory* HFactory;

    /**
     * Preloader handle
     */
    typedef struct ResourcePreloader* HPreloader;
    typedef struct PreloadHintInfo* HPreloadHintInfo;

    typedef uintptr_t ResourceType;

    /**
     * Parameters to ResourcePreload callback.
     */
    struct ResourcePreloadParams
    {
        /// Factory handle
        HFactory m_Factory;
        /// Resource context
        void* m_Context;
        /// File name of the loaded file
        const char* m_Filename;
        /// Buffer containing the loaded file
        const void* m_Buffer;
        /// Size of data buffer
        uint32_t m_BufferSize;
        /// Hinter info. Use this when calling PreloadHint
        HPreloadHintInfo m_HintInfo;
        /// Writable user data that will be passed on to ResourceCreate function
        void** m_PreloadData;
    };

    /**
     * Resource preloading function. This may be called from a separate loading thread
     * but will not keep any mutexes held while executing the call. During this call
     * PreloadHint can be called with the supplied hint_info handle.
     * If RESULT_OK is returned, the resource Create function is guaranteed to be called
     * with the preload_data value supplied.
     * @param param Resource preloading parameters
     * @return RESULT_OK on success
     */
    typedef Result (*FResourcePreload)(const ResourcePreloadParams& params);

    /**
     * Parameters to ResourceCreate callback.
     */
    struct ResourceCreateParams
    {
        /// Factory handle
        HFactory m_Factory;
        /// Resource context
        void* m_Context;
        /// File name of the loaded file
        const char* m_Filename;
        /// Buffer containing the loaded file
        const void* m_Buffer;
        /// Size of the data buffer
        uint32_t m_BufferSize;
        /// Preloaded data from Preload phase
        void* m_PreloadData;
        /// Resource descriptor to fill in
        SResourceDescriptor* m_Resource;
    };

    /**
     * Resource create function
     * @param params Resource creation arguments
     * @return CREATE_RESULT_OK on success
     */
    typedef Result (*FResourceCreate)(const ResourceCreateParams& params);

    /**
     * Parameters to ResourcePostCreate callback.
     */
    struct ResourcePostCreateParams
    {
        /// Factory handle
        HFactory m_Factory;
        /// Resource context
        void* m_Context;
        /// Preloaded data from Preload phase
        void* m_PreloadData;
        /// Resource descriptor passed from create function
        SResourceDescriptor* m_Resource;
    };

    /**
     * Resource postcreate function
     * @param params Resource postcreation arguments
     * @return CREATE_RESULT_OK on success or CREATE_RESULT_PENDING when pending
     * @note returning CREATE_RESULT_PENDING will result in a repeated callback the following update.
     */
    typedef Result (*FResourcePostCreate)(const ResourcePostCreateParams& params);

    /**
     * Parameters to ResourceDestroy callback.
     */
    struct ResourceDestroyParams
    {
        /// Factory handle
        HFactory m_Factory;
        /// Resource context
        void* m_Context;
        /// Resource descriptor for resource to destroy
        SResourceDescriptor* m_Resource;
    };

    /**
     * Resource destroy function
     * @param params Resource destruction arguments
     * @return CREATE_RESULT_OK on success
     */
    typedef Result (*FResourceDestroy)(const ResourceDestroyParams& params);

    /**
     * Parameters to ResourceRecreate callback.
     */
    struct ResourceRecreateParams
    {
        /// Factory handle
        HFactory m_Factory;
        /// Resource context
        void* m_Context;
        /// File name hash of the data
        uint64_t m_NameHash;
        /// File name of the loaded file
        const char* m_Filename;
        /// Data buffer containing the loaded file
        const void* m_Buffer;
        /// Size of data buffer
        uint32_t m_BufferSize;
        /// Pointer holding a precreated message
        const void* m_Message;
        /// Resource descriptor to write into
        SResourceDescriptor* m_Resource;
    };

    /**
     * Resource recreate function. Recreate resource in-place.
     * @params params Parameters for resource creation
     * @return CREATE_RESULT_OK on success
     */
    typedef Result (*FResourceRecreate)(const ResourceRecreateParams& params);

    /**
     * Parameters to ResourceReloaded callback.
     */
    struct ResourceReloadedParams
    {
        /// User data supplied when the callback was registered
        void* m_UserData;
        /// Descriptor of the reloaded resource
        SResourceDescriptor* m_Resource;
        /// Name of the resource, same as provided to Get() when the resource was obtained
        const char* m_Name;
        /// Hashed name of the resource
        uint64_t m_NameHash;
    };

    /**
     * Function called when a resource has been reloaded.
     * @param params Parameters
     * @see RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT
     * @see RegisterResourceReloadedCallback
     * @see Get
     */
    typedef void (*ResourceReloadedCallback)(const ResourceReloadedParams& params);

    /**
     * Parameters to PreloaderCompleteCallback.
     */
    struct PreloaderCompleteCallbackParams
    {
        /// Factory handle
        HFactory m_Factory;
        /// User data supplied when the callback was registered
        void* m_UserData;
    };

    /**
     * Function called by UpdatePreloader when preoloading is complete and before postcreate callbacks are processed.
     * @param PreloaderCompleteCallbackParams parameters passed to callback function
     * @return true if succeeded
     * @see UpdatePreloader
     */
    typedef bool (*FPreloaderCompleteCallback)(const PreloaderCompleteCallbackParams* params);

    /**
     * Set default NewFactoryParams params
     * @param params
     */
    void SetDefaultNewFactoryParams(struct NewFactoryParams* params);

    struct EmbeddedResource
    {
        EmbeddedResource() : m_Data(NULL), m_Size(0)
        {

        }

        // Pointer to a resource. Set to NULL for no resource (default value)
        const void* m_Data;

        // Size of resource.
        uint32_t    m_Size;
    };

    /**
     * New factory parmetes
     */
    struct NewFactoryParams
    {
        /// Maximum number of resource in factory. Default is 1024
        uint32_t m_MaxResources;

        /// Factory flags. Default is RESOURCE_FACTORY_FLAGS_EMPTY
        uint32_t m_Flags;

        EmbeddedResource m_ArchiveIndex;
        EmbeddedResource m_ArchiveData;
        EmbeddedResource m_ArchiveManifest;

        uint32_t m_Reserved[5];

        NewFactoryParams()
        {
            SetDefaultNewFactoryParams(this);
        }
    };

    /**
     * Create a new factory
     * @param params New factory parameters
     * @param uri Resource root uri, e.g. http://locahost:5000 or build/default/content
     * @return Factory handle. NULL if out of memory or invalid uri.
     */
    HFactory NewFactory(NewFactoryParams* params, const char* uri);

    /**
     * Delete a factory
     * @param factory Factory handle
     */
    void DeleteFactory(HFactory factory);

    /**
     * Update resource factory. Required to be called periodically when http server support is enabled.
     * @param factory Factory handle
     */
    void UpdateFactory(HFactory factory);

    /**
     * Register a resource type
     * @param factory Factory handle
     * @param extension File extension for resource
     * @param context User context
     * @param preload_function Preload function. Optional, 0 if no preloading is used
     * @param create_function Create function pointer
     * @param post_create_function Post create function pointer
     * @param destroy_function Destroy function pointer
     * @param recreate_function Recreate function pointer. Optional, 0 if recreate is not supported.
     * @return RESULT_OK on success
     */
    Result RegisterType(HFactory factory,
                               const char* extension,
                               void* context,
                               FResourcePreload preload_function,
                               FResourceCreate create_function,
                               FResourcePostCreate post_create_function,
                               FResourceDestroy destroy_function,
                               FResourceRecreate recreate_function);

    /**
     * Get a resource from factory
     * @param factory Factory handle
     * @param name Resource name
     * @param resource Created resource
     * @return RESULT_OK on success
     */
    Result Get(HFactory factory, const char* name, void** resource);

    /**
     * Find a resource by a canonical path hash.
     * @param factory Factory handle
     * @param path_hash Resource path hash
     * @return SResourceDescriptor* pointer to the resource descriptor
     */
    SResourceDescriptor* FindByHash(HFactory factory, uint64_t canonical_path_hash);

    /**
     * Get raw resource data. Unregistered resources can be loaded with this function.
     * The returned resource data must be deallocated with free()
     * @param factory Factory handle
     * @param name Resource name
     * @param resource Resource data
     * @param resource_size Resource size
     * @return RESULT_OK on success
     */
    Result GetRaw(HFactory factory, const char* name, void** resource, uint32_t* resource_size);

    /**
     * Updates a preexisting resource with new data
     * @param factory Factory handle
     * @param hashed_name The hashed canonical name (E.g. hash("/my/icon.texturec") or hash("/my/icon.texturec_123"))
     * @param buffer The buffer
     * @return RESULT_OK on success
     */
    Result SetResource(HFactory factory, uint64_t hashed_name, void* buffer, uint32_t size);

    /**
     * Updates a preexisting resource with new data
     * @param factory Factory handle
     * @param hashed_name The hashed canonical name (E.g. hash("/my/icon.texturec") or hash("/my/icon.texturec_123"))
     * @param buffer The buffer
     * @return RESULT_OK on success
     */
    Result SetResource(HFactory factory, uint64_t hashed_name, void* message);


    /**
     * Reload a specific resource
     * @param factory Resource factory
     * @param name Name that identifies the resource, i.e. the same name used in Get
     * @param out_descriptor The resource descriptor as an output argument. It will not be written to if null, otherwise it will always be written to.
     * @see Get
     */
    Result ReloadResource(HFactory factory, const char* name, SResourceDescriptor** out_descriptor);

    /**
     * Get type for resource
     * @param factory Factory handle
     * @param resource Resource
     * @param type Returned type
     * @return RESULT_OK on success
     */
    Result GetType(HFactory factory, void* resource, ResourceType* type);

    /**
     * Get type from extension
     * @param factory Factory handle
     * @param extension File extension
     * @param type Returned type
     * @return RESULT_OK on success
     */
    Result GetTypeFromExtension(HFactory factory, const char* extension, ResourceType* type);

    /**
     * Get extension from type
     * @param factory Factory handle
     * @param type Resource type
     * @param extension Returned extension
     * @return RESULT_OK on success
     */
    Result GetExtensionFromType(HFactory factory, ResourceType type, const char** extension);

    /**
     * Get resource descriptor from resource (name)
     * @param factory Factory handle
     * @param name Resource name
     * @param descriptor Returned resource descriptor
     * @return RESULT_OK on success
     */
    Result GetDescriptor(HFactory factory, const char* name, SResourceDescriptor* descriptor);

    /**
     * Get resource descriptor from resource (hash) with supplied extensions
     * @param factory Factory handle
     * @param hashed_name Resource name hash
     * @param exts Allowed extension hashes
     * @param ext_count Count of exts
     * @param descriptor pointer to write result to in case of RESULT_OK
     * @return RESULT_OK on success
     */
    Result GetDescriptorWithExt(HFactory factory, uint64_t hashed_name, const uint64_t* exts, uint32_t ext_count, SResourceDescriptor* descriptor);

    /**
     * Increase resource reference count
     * @param factory Factory handle
     * @param resource Resource
     */
    void IncRef(HFactory factory, void* resource);

    /**
     * Release resource
     * @param factory Factory handle
     * @param resource Resource
     */
    void Release(HFactory factory, void* resource);

    /**
     * Register a callback function that will be called with the specified user data when a resource has been reloaded.
     * The callbacks will not necessarily be called in the order they were registered.
     * This has only effect when reloading is supported.
     * @param factory Handle of the factory to which the callback will be registered
     * @param callback Callback function to register
     * @param user_data User data that will be supplied to the callback when it is called
     * @see RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT
     */
    void RegisterResourceReloadedCallback(HFactory factory, ResourceReloadedCallback callback, void* user_data);

    /**
     * Remove a registered callback function, O(n).
     * @param factory Handle of the factory from which the callback will be removed
     * @param callback Callback function to remove
     * @param user_data User data that was supplied when the callback was registered
     * @see RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT
     */
    void UnregisterResourceReloadedCallback(HFactory factory, ResourceReloadedCallback callback, void* user_data);

    /**
     * Create a new preloader
     * @param factory Factory handle
     * @param name Resource to load
     * @return CREATE_RESULT_OK on success
     */
    HPreloader NewPreloader(HFactory factory, const char* name);

    /**
     * Create a new preloader
     * @param factory Factory handle
     * @param array of names of resources to load
     * @return CREATE_RESULT_OK on success
     */
    HPreloader NewPreloader(HFactory factory, const dmArray<const char*>& names);

    /**
     * Perform one update tick of the preloader, with a soft time limit for
     * how much time to spend.
     * @param preloader Preloader
     * @param complete_callback Preloader complete callback
     * @param complete_callback_params PreloaderCompleteCallbackParams passed to the complete callback
     * @param soft_time_limit Time limit in us
     * @return RESULT_PENDING while still loading, otherwise resource load result.
     */
    Result UpdatePreloader(HPreloader preloader, FPreloaderCompleteCallback complete_callback, PreloaderCompleteCallbackParams* complete_callback_params, uint32_t soft_time_limit);

    /**
     * Destroy the preloader. Note that currently it will spin and block until
     * all loads have completed, if there still are any pending.
     * @param preloader Preloader
     * @return RESULT_PENDING while still loading, otherwise resource load result.
     */
    void DeletePreloader(HPreloader preloader);

    /**
     * Hint the preloader what to load before Create is called on the resource.
     * The resources are not guaranteed to be loaded before Create is called.
     * This function can be called from a worker thread.
     * @param name Resource name
     * @return bool if successfully invoking preloader.
     */
    bool PreloadHint(HPreloadHintInfo preloader, const char *name);

    Manifest* GetManifest(HFactory factory);

    Result LoadArchiveIndex(const char* bundle_dir, HFactory factory);

    Result ManifestLoadMessage(uint8_t* manifest_msg_buf, uint32_t size, dmResource::Manifest*& out_manifest);

    Result StoreManifest(Manifest* manifest);

    /**
     * Verify that all resources the manifest expects to be bundled actually are bundled.
     */
    Result VerifyResourcesBundled(HFactory factory, Manifest* manifest);

    /**
     * Loads the public RSA key from the bundle.
     * Uses the public key to decrypt the manifest signature to get the content hash.
     * Compares the decrypted content hash to the expected content hash.
     * Diagram of what to do; https://crypto.stackexchange.com/questions/12768/why-hash-the-message-before-signing-it-with-rsa
     * Inspect asn1 key content; http://lapo.it/asn1js/#
     */
    Result VerifyManifestHash(HFactory factory, Manifest* manifest, const uint8_t* expected_digest, uint32_t expected_len);

    /**
     * Create new archive index with resource.
     * @param manifest Manifest to use
     * @param hash_digest Hash digest length buffer
     * @param hash_digest_length Hash digest length
     * @param resource LiveUpdate resource to create with
     * @param out_new_index New archive index
     * @return RESULT_OK on success
     */
    Result NewArchiveIndexWithResource(Manifest* manifest, const uint8_t* hash_digest, uint32_t hash_digest_length, const dmResourceArchive::LiveUpdateResource* resource, const char* proj_id, dmResourceArchive::HArchiveIndex& out_new_index);

    /**
     * Determines if the resource could be unique
     * @param name Resource name
    */
    bool IsPathTagged(const char* name);


    /**
     * Returns the canonical path hash of a resource
     * @param factory Factory handle
     * @param resource Resource
     * @param hash Returned hash
     * @return RESULT_OK on success
    */
    Result GetPath(HFactory factory, const void* resource, uint64_t* hash);

    /**
     * Returns the mutex held when loading asynchronous
     * @param factory Factory handle
     * @return Mutex pointer
    */
    dmMutex::HMutex GetLoadMutex(const dmResource::HFactory factory);

    /**
     * Releases the builtins manifest
     * Use when it's no longer needed, e.g. the user project loaded properly
     */
    void ReleaseBuiltinsManifest(HFactory factory);

    /**
     * Returns the length in bytes of the supplied hash algorithm
     */
    uint32_t HashLength(dmLiveUpdateDDF::HashAlgorithm algorithm);

    /**
     * Converts the supplied byte buffer to hexadecimal string representation
     * @param byte_buf The byte buffer
     * @param byte_buf_len The byte buffer length
     * @param out_buf The output string buffer
     * @param out_len The output buffer length
     */
    void BytesToHexString(const uint8_t* byte_buf, uint32_t byte_buf_len, char* out_buf, uint32_t out_len);

    /**
     * Byte-wise comparison of two hash buffers
     * @param digest The hash digest to compare
     * @param len The hash digest length
     * @param buf The expected hash digest
     * @param buflen The expected hash digest length
     * @return RESULT_OK if the hashes are equal in length and content
     */
    Result HashCompare(const uint8_t* digest, uint32_t len, const uint8_t* expected_digest, uint32_t expected_len);

    // Platform specific implementation of archive and manifest loading. Data written into mount_info must
    // be provided for unloading and may contain information about memory mapping etc.
    Result MountArchiveInternal(const char* index_path, const char* data_path, const char* lu_data_path, dmResourceArchive::HArchiveIndexContainer* archive, void** mount_info);
    void UnmountArchiveInternal(dmResourceArchive::HArchiveIndexContainer &archive, void* mount_info);
    Result MountManifest(const char* manifest_filename, void*& out_map, uint32_t& out_size);
    Result UnmountManifest(void *& map, uint32_t size);
    // Files mapped with this function should be unmapped with UnmapFile(...)
    Result MapFile(const char* filename, void*& map, uint32_t& size);
    Result UnmapFile(void*& map, uint32_t size);

    /**
     * Struct returned from the resource iterator api
     */
    struct IteratorResource
    {
        dmhash_t m_Id;          // The name of the resource
        uint32_t m_SizeOnDisc;  // The size on disc (i.e. in the .darc file)
        uint32_t m_Size;        // in memory size, may be 0
        uint32_t m_RefCount;    // The current ref count
    };

    typedef bool (*FResourceIterator)(const IteratorResource& resource, void* user_ctx);

    /**
     * Iterates over all loaded resources, and invokes the callback function with the resource informations
     * @param factory   The resource factory holding all resources
     * @param callback  The callback function which is invoked for each resources.
                        It should return true if the iteration should continue, and false otherwise.
     * @param user_ctx  The user defined context which is passed along with each callback
     */
    void IterateResources(HFactory factory, FResourceIterator callback, void* user_ctx);

    /**
     */
    const char* ResultToString(Result result);
}

#endif // RESOURCE_H
