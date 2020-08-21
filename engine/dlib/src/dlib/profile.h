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

#ifndef DM_PROFILE_H
#define DM_PROFILE_H

#include <stdint.h>
#include <dlib/array.h>
#include <dlib/log.h>
#include <dlib/atomic.h>
#include <dlib/dlib.h>
#include <dlib/dstrings.h>
#include <dlib/static_assert.h> // ANALYZE_USE_POINTER

#define DM_PROFILE_PASTE(x, y) x ## y
#define DM_PROFILE_PASTE2(x, y) DM_PROFILE_PASTE(x, y)

/**
 * Profiler string internalize macro
 * name is the string to internalize
 * returns the internalized string pointer or zero if profiling is disabled
 */
#define DM_INTERNALIZE(name)
#undef DM_INTERNALIZE

/**
 * Profile macro.
 * scope_name is the scope name. Must be a literal
 * name is the sample name. Must be literal, to use non-literal name use DM_PROFILE_DYN
 */
#define DM_PROFILE(scope_name, name)
#undef DM_PROFILE

/**
 * Profile macro.
 * scope_name is the scope name. Must be a literal
 * name is the sample name. Can be non-literal, use DM_PROFILE if you know it is a literal
 * name_hash is the hash of the sample name obtained via dmProfile::GetNameHash()
 */
#define DM_PROFILE_DYN(scope_name, name, name_hash)
#undef DM_PROFILE_DYN

/**
 * Profile counter macro
 * name is the counter name. Must be a literal
 * amount is the amount (integer) to add to the specific counter.
 */
#define DM_COUNTER(name, amount)
#undef DM_COUNTER

/**
 * Profile counter macro for non-literal strings, caller must provide hash
 * counter_index is counter index from AllocateCounter.
 * amount is the amount (integer) to add to the specific counter.
 */
#define DM_COUNTER_DYN(counter_index, amount)
#undef DM_COUNTER_DYN

#if defined(NDEBUG)
    #define DM_INTERNALIZE(name) 0
    #define DM_PROFILE_SCOPE(scope_instance_name, name)
    #define DM_PROFILE(scope_name, name)
    #define DM_PROFILE_DYN(scope_name, name, name_hash)
    #define DM_COUNTER(name, amount)
    #define DM_COUNTER_DYN(counter_index, amount)
#else
    #define DM_INTERNALIZE(name) \
        (dmProfile::g_IsInitialized ? dmProfile::Internalize(name, (uint32_t)strlen(name), dmProfile::GetNameHash(name, (uint32_t)strlen(name))) : 0)

    #define DM_PROFILE_SCOPE(scope_index, name, name_hash) \
        dmProfile::ProfileScope DM_PROFILE_PASTE2(profile_scope, __LINE__)(scope_index, name, name_hash);

    #define DM_PROFILE(scope_name, name) \
        static const uint32_t DM_PROFILE_PASTE2(scope_index, __LINE__) = dmProfile::g_IsInitialized ? dmProfile::AllocateScope(#scope_name) : 0xffffffffu; \
        static const uint32_t DM_PROFILE_PASTE2(hash, __LINE__)        = dmProfile::g_IsInitialized ? dmProfile::GetNameHash(name, (uint32_t)strlen(name)) : 0; \
        DM_PROFILE_SCOPE(DM_PROFILE_PASTE2(scope_index, __LINE__), name, DM_PROFILE_PASTE2(hash, __LINE__))

    #define DM_PROFILE_DYN(scope_name, name, name_hash) \
        static const uint32_t DM_PROFILE_PASTE2(scope_index, __LINE__) = dmProfile::g_IsInitialized ? dmProfile::AllocateScope(#scope_name) : 0xffffffffu; \
        DM_PROFILE_SCOPE(DM_PROFILE_PASTE2(scope_index, __LINE__), name, name_hash)

    #define DM_COUNTER(name, amount) \
        static uint32_t DM_PROFILE_PASTE2(counter_index, __LINE__) = dmProfile::g_IsInitialized ? dmProfile::AllocateCounter(name) : 0xffffffffu; \
        if (DM_PROFILE_PASTE2(counter_index, __LINE__) != 0xffffffffu) { \
            dmProfile::AddCounterIndex(DM_PROFILE_PASTE2(counter_index, __LINE__), amount);\
        }

    #define DM_COUNTER_DYN(counter_index, amount) \
        if (counter_index != 0xffffffffu) { \
            dmProfile::AddCounterIndex(counter_index, amount); \
        }
#endif

namespace dmProfile
{
    /// Profile snapshot handle
    typedef struct Profile* HProfile;

    struct ScopeData;

    /**
     * Profile scope
     */
    struct Scope
    {
        /// Scope name
        const char* m_Name;
        /// Scope name hash
        uint32_t    m_NameHash;
        /// Scope index, range [0, scopes-1]
        uint16_t    m_Index;
        /// Internal data
        void*       m_Internal;
    };

    /**
     * Scope data
     */
    struct ScopeData
    {
        /// The scope
        Scope*   m_Scope;
        /// Total time spent in scope (in ticks) summed over all threads
        uint32_t m_Elapsed;
        /// Occurrences of this scope summed over all threads
        uint32_t m_Count;
    };

    /**
     * Profile sample
     */
    struct Sample
    {
        /// Sample name
        const char* m_Name;
        /// Sampled within scope
        Scope*      m_Scope;
        /// Start time in ticks
        uint32_t    m_Start;
        /// Elapsed time in ticks
        uint32_t    m_Elapsed;
        /// Sample name hash
        uint32_t    m_NameHash;
        /// Thread id this sample belongs to
        uint16_t    m_ThreadId;
        /// Padding to 64-bit align
        uint16_t    m_Pad;
    };

    /**
     * Profile counter
     */
    struct Counter
    {
        /// Counter name
        const char*      m_Name;
        /// Counter name hash
        uint32_t         m_NameHash;
    };

    /**
     * Profile counter data
     */
    struct CounterData
    {
        /// The counter
        Counter*       m_Counter;
        /// Counter value
        int32_atomic_t m_Value;
    };

    /**
     * Initialize profiler
     * @param max_scopes Maximum scopes
     * @param max_samples Maximum samples
     * @param max_counters Maximum counters
     */
    void Initialize(uint32_t max_scopes, uint32_t max_samples, uint32_t max_counters);

    /**
     * Finalize profiler
     */
    void Finalize();

    /**
     * Begin profiling, eg start of frame
     * @note NULL is returned if profiling is disabled
     * @return A snapshot of the current profile. Must be release by #Release after processed. It's valid to keep the profile snapshot throughout a "frame".
     */
    HProfile Begin();

    /**
     * Pause profiling
     * @param pause True to pause. False to resume.
     */
    void Pause(bool pause);

    /**
     * Release profile returned by #Begin
     * @param profile Profile to release
     */
    void Release(HProfile profile);

    /**
     * Get ticks per second
     * @return Ticks per second
     */
    uint64_t GetTicksPerSecond();

    /**
     * Iterate over all registered strings
     * @param profile Profile snapshot to iterate over
     * @param context User context
     * @param call_back Call-back function pointer
     */
    void IterateStrings(HProfile profile, void* context, void (*call_back)(void* context, const uintptr_t* key, const char** value));

    /**
     * Iterate over all scopes
     * @param profile Profile snapshot to iterate over
     * @param context User context
     * @param call_back Call-back function pointer
     */
    void IterateScopes(HProfile profile, void* context, void (*call_back)(void* context, const Scope* scope_data));

    /**
     * Iterate over all scopes
     * @param profile Profile snapshot to iterate over
     * @param context User context
     * @param sort sort the entries
     * @param call_back Call-back function pointer
     */
    void IterateScopeData(HProfile profile, void* context, bool sort, void (*call_back)(void* context, const ScopeData* scope_data));

    /**
     * Iterate over all samples
     * @param profile Profile snapshot to iterate over
     * @param context User context
     * @param sort sort the entries
     * @param call_back Call-back function pointer
     */
    void IterateSamples(HProfile profile, void* context, bool sort, void (*call_back)(void* context, const Sample* sample));

    /**
     * Iterate over all counters
     * @param profile Profile snapshot to iterate over
     * @param context User context
     * @param call_back Call-back function pointer
     */
    void IterateCounters(HProfile profile, void* context, void (*call_back)(void* context, const Counter* counter));

    /**
     * Iterate over all counters
     * @param profile Profile snapshot to iterate over
     * @param context User context
     * @param call_back Call-back function pointer
     */
    void IterateCounterData(HProfile profile, void* context, void (*call_back)(void* context, const CounterData* counter));

    /**
     * Internal function
     * @param name
     * @return global scope index
     */
    uint32_t AllocateScope(const char* name);

    /**
     * Internal function
     * @return #Sample
     */
    Sample* AllocateSample();

    /**
     * Create an internalized string. Use this function in DM_PROFILE if the
     * name isn't valid for the life-time of the application
     * @param string string to internalize
     * @param string_length the length of the string to internalize
     * @param string_hash the hash of the string to internalize
     * @return internalized string or 0 if profiling is not enabled
     */
    const char* Internalize(const char* string, uint32_t string_length, uint32_t string_hash);

    /**
     * Generates a hash for the name
     * @param name string to hash
     * @param string_length length of the name to hash 
     * @return the hash or 0 if profiling is not enabled
     */
    uint32_t GetNameHash(const char* name, uint32_t string_length);

    /**
     * Add #amount to counter with #name
     * @param name Counter name
     * @param amount Amount to add
     */
    void AddCounter(const char* name, uint32_t amount);

    /**
     * Creates a counter and returns the global index for the counter
     * @param name Counter name
     * @return the counter index
     */  
    uint32_t AllocateCounter(const char* name);

    /**
     * Add #amount to counter at the counter index. Faster version of #AddCounter
     * @param counter_index Global counter index obtained with #AllocateCounter
     * @param amount Amount to add
     */
    void AddCounterIndex(uint32_t counter_index, uint32_t amount);

    /**
     * Get time for the frame total
     * @return Total frame time
     */
    float GetFrameTime();

    /**
     * Get time for the frame total during the last 60 frames
     * @return Total frame time
     */
    float GetMaxFrameTime();

    /**
     * Check of out of scope resources
     * @return True if out of scope resources
     */
    bool IsOutOfScopes();

    /**
     * Check of out of sample resources
     * @return True if out of sample resources
     */
    bool IsOutOfSamples();

    /// Internal, do not use.
    extern bool g_IsInitialized;

    uint64_t GetNowTicks();

    /// Internal, do not use.
    struct ProfileScope
    {
        Sample* m_Sample;
        uint64_t m_StartTick;
        inline ProfileScope(uint32_t scope_index, const char* name, uint32_t name_hash)
        {
            if (scope_index != 0xffffffffu)
            {
                StartScope(scope_index, name, name_hash);
            }
            else
            {
                m_Sample = 0;
            }
        }

        inline ~ProfileScope()
        {
            if (m_Sample)
            {
                EndScope();
            }
        }

        void StartScope(uint32_t scope_index, const char* name, uint32_t name_hash);
        void EndScope();
    };

    uint32_t GetTickSinceBegin();

} // namespace dmProfile

#endif
