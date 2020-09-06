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

#ifndef DMSDK_SCRIPT_H
#define DMSDK_SCRIPT_H

#include <stdint.h>
#include <stdarg.h>
#include <dmsdk/dlib/buffer.h>

extern "C"
{
#include <dmsdk/lua/lua.h>
#include <dmsdk/lua/lauxlib.h>
}

namespace Vectormath {
    namespace Aos {
        class Vector3;
        class Vector4;
        class Quat;
        class Matrix4;
    }
}
namespace dmJson {
    struct Document;
}

namespace dmScript
{
    /*# SDK Script API documentation
     * [file:<dmsdk/script/script.h>]
     *
     * Built-in scripting functions.
     *
     * @document
     * @name Script
     * @namespace dmScript
     */

    /**
    * LuaStackCheck struct. Internal
    *
    * LuaStackCheck utility to make sure we check the Lua stack state before leaving a function.
    * m_Diff is the expected difference of the stack size.
    *
    */
    struct LuaStackCheck
    {
        LuaStackCheck(lua_State* L, int diff);
        ~LuaStackCheck();
        void Verify(int diff);
        #if defined(__GNUC__)
        int Error(const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));;
        #else
        int Error(const char* fmt, ...);
        #endif

        /// The Lua state to check
        lua_State* m_L;
        /// The current top of the Lua stack (from lua_gettop())
        int m_Top;
        /// The expected difference in stack size when this sctruct goes out of scope
        int m_Diff;
    };


    /*# helper macro to validate the Lua stack state before leaving a function.
     *
     * Diff is the expected difference of the stack size.
     * If luaL_error, or another function that executes a long-jump, is part of the executed code,
     * the stack guard cannot be guaranteed to execute at the end of the function.
     * In that case you should manually check the stack using `lua_gettop`.
     * In the case of luaL_error, see [ref:DM_LUA_ERROR].
     *
     * @macro
     * @name DM_LUA_STACK_CHECK
     * @param L [type:lua_State*] lua state
     * @param diff [type:int] Number of expected items to be on the Lua stack once this struct goes out of scope
     * @examples
     *
     * ```cpp
     * DM_LUA_STACK_CHECK(L, 1);
     * lua_pushnumber(L, 42);
     * ```
     */
    #define DM_LUA_STACK_CHECK(_L_, _diff_)     dmScript::LuaStackCheck _DM_LuaStackCheck(_L_, _diff_);


    /*# helper macro to validate the Lua stack state and throw a lua error.
     *
     * This macro will verify that the Lua stack size hasn't been changed before
     * throwing a Lua error, which will long-jump out of the current function.
     * This macro can only be used together with [ref:DM_LUA_STACK_CHECK] and should
     * be prefered over manual checking of the stack.
     *
     * @macro
     * @name DM_LUA_ERROR
     * @param fmt [type:const char*] Format string that contains error information.
     * @param args [type:...] Format string args (variable arg list)
     * @examples
     *
     * ```cpp
     * static int ModuleFunc(lua_State* L)
     * {
     *     DM_LUA_STACK_CHECK(L, 1);
     *     if (some_error_check(L))
     *     {
     *         return DM_LUA_ERROR("some error message");
     *     }
     *     lua_pushnumber(L, 42);
     *     return 1;
     * }
     * ```
     */
    #define DM_LUA_ERROR(_fmt_, ...)   _DM_LuaStackCheck.Error(_fmt_,  ##__VA_ARGS__); \


    /*# wrapper for luaL_ref.
     *
     * Creates and returns a reference, in the table at index t, for the object at the
     * top of the stack (and pops the object).
     * It also tracks number of global references kept.
     *
     * @name dmScript::Ref
     * @param L [type:lua_State*] lua state
     * @param table [type:int] table the lua table that stores the references. E.g LUA_REGISTRYINDEX
     * @return reference [type:int] the new reference
     */
    int Ref(lua_State* L, int table);

    /*# wrapper for luaL_unref.
     *
     * Releases reference ref from the table at index t (see luaL_ref).
     * The entry is removed from the table, so that the referred object can be collected.
     * It also decreases the number of global references kept
     *
     * @name dmScript::Unref
     * @param L [type:lua_State*] lua state
     * @param table [type:int] table the lua table that stores the references. E.g LUA_REGISTRYINDEX
     * @param reference [type:int] the reference to the object
     */
    void Unref(lua_State* L, int table, int reference);

    /*#
     * Retrieve current script instance from the global table and place it on the top of the stack, only valid when set.
     * (see [ref:dmScript::GetMainThread])
     * @name dmScript::GetInstance
     * @param L [type:lua_State*] lua state
     */
    void GetInstance(lua_State* L);

    /*#
     * Sets the current script instance
     * Set the value on the top of the stack as the instance into the global table and pops it from the stack.
     * (see [ref:dmScript::GetMainThread])
     * @name dmScript::SetInstance
     * @param L [type:lua_State*] lua state
     */
    void SetInstance(lua_State* L);

    /*#
     * Check if the script instance in the lua state is valid. The instance is assumed to have been previously set by [ref:dmScript::SetInstance].
     * @name dmScript::IsInstanceValid
     * @param L [type:lua_State*] lua state
     * @return boolean [type:bool] Returns true if the instance is valid
     */
    bool IsInstanceValid(lua_State* L);

    /*#
     * Retrieve the main thread lua state from any lua state (main thread or coroutine).
     * @name dmScript::GetMainThread
     * @param L [type:lua_State*] lua state
     * @return lua_State [type:lua_State*] the main thread lua state
     *
     * @examples
     *
     * How to create a Lua callback
     *
     * ```cpp
     * struct LuaCallbackInfo
     * {
     *     LuaCallbackInfo() : m_L(0), m_Callback(LUA_NOREF), m_Self(LUA_NOREF) {}
     *     lua_State* m_L;
     *     int        m_Callback;
     *     int        m_Self;
     * };
     *
     * static void RegisterCallback(lua_State* L, int index, LuaCallbackInfo* cbk)
     * {
     *     if(cbk->m_Callback != LUA_NOREF)
     *     {
     *         dmScript::Unref(cbk->m_L, LUA_REGISTRYINDEX, cbk->m_Callback);
     *         dmScript::Unref(cbk->m_L, LUA_REGISTRYINDEX, cbk->m_Self);
     *     }
     *
     *     cbk->m_L = dmScript::GetMainThread(L);
     *
     *     luaL_checktype(L, index, LUA_TFUNCTION);
     *     lua_pushvalue(L, index);
     *     cbk->m_Callback = dmScript::Ref(L, LUA_REGISTRYINDEX);
     *
     *     dmScript::GetInstance(L);
     *     cbk->m_Self = dmScript::Ref(L, LUA_REGISTRYINDEX);
     * }
     *
     * static void UnregisterCallback(LuaCallbackInfo* cbk)
     * {
     *     if(cbk->m_Callback != LUA_NOREF)
     *     {
     *         dmScript::Unref(cbk->m_L, LUA_REGISTRYINDEX, cbk->m_Callback);
     *         dmScript::Unref(cbk->m_L, LUA_REGISTRYINDEX, cbk->m_Self);
     *         cbk->m_Callback = LUA_NOREF;
     *     }
     * }
     *
     * LuaCallbackInfo g_MyCallbackInfo;
     *
     * static void InvokeCallback(LuaCallbackInfo* cbk)
     * {
     *     if(cbk->m_Callback == LUA_NOREF)
     *     {
     *         return;
     *     }
     *
     *     lua_State* L = cbk->m_L;
     *     int top = lua_gettop(L);
     *
     *     lua_rawgeti(L, LUA_REGISTRYINDEX, cbk->m_Callback);
     *
     *     // Setup self (the script instance)
     *     lua_rawgeti(L, LUA_REGISTRYINDEX, cbk->m_Self);
     *     lua_pushvalue(L, -1);
     *
     *     dmScript::SetInstance(L);
     *
     *     lua_pushstring(L, "Hello from extension!");
     *     lua_pushnumber(L, 76);
     *
     *     int number_of_arguments = 3; // instance + 2
     *     int ret = lua_pcall(L, number_of_arguments, 0, 0);
     *     if(ret != 0) {
     *         dmLogError("Error running callback: %s", lua_tostring(L, -1));
     *         lua_pop(L, 1);
     *     }
     *     assert(top == lua_gettop(L));
     * }
     *
     * static int Start(lua_State* L)
     * {
     *     DM_LUA_STACK_CHECK(L, 0);
     *
     *     RegisterCallback(L, 1, &g_MyCallbackInfo);
     *
     *     return 0;
     * }
     *
     * static int Update(lua_State* L)
     * {
     *     DM_LUA_STACK_CHECK(L, 0);
     *
     *     static int count = 0;
     *     if( count++ == 5 )
     *     {
     *         InvokeCallback(&g_MyCallbackInfo);
     *         UnregisterCallback(&g_MyCallbackInfo);
     *     }
     *     return 0;
     * }
     * ```
     */
    lua_State* GetMainThread(lua_State* L);


    enum LuaBufferOwnership
    {
        OWNER_C   = 0,
        OWNER_LUA = 1,
        OWNER_RES = 2,
    };

    /*# Lua wrapper for a dmBuffer::HBuffer
     *
     * Holds info about the buffer and who owns it.
     *
     * @struct
     * @name dmScript::LuaHBuffer
     * @member m_Buffer [type:dmBuffer::HBuffer]            The buffer (or resource)
     * @member m_Owner  [type:dmScript::LuaBufferOwnership] What ownership the pointer has
     */
    struct LuaHBuffer
    {
        union {
            dmBuffer::HBuffer   m_Buffer;
            void*               m_BufferRes;
        };

        /// Specifies the owner of the buffer.
        /// OWNER_C   - m_Buffer is owned by C side, should not be destroyed when GCed
        /// OWNER_LUA - m_Buffer is owned by Lua side, will be destroyed when GCed
        /// OWNER_RES - m_Buffer not used, has a reference to a buffer resource instead. m_BufferRes is owned by C side, will be released when GCed
        union {
            bool                m_UseLuaGC; // Deprecated
            LuaBufferOwnership  m_Owner;
        };
    };

    /*# check if the value is a dmScript::LuaHBuffer
     *
     * Check if the value is a dmScript::LuaHBuffer
     *
     * @name dmScript::IsBuffer
     * @param L [type:lua_State*] lua state
     * @param index [type:int] Index of the value
     * @return boolean [type:boolean] True if value at index is a LuaHBuffer
     */
    bool IsBuffer(lua_State* L, int index);

    /*# push a LuaHBuffer onto the supplied lua state
     *
     * Will increase the stack by 1.
     *
     * @name dmScript::PushBuffer
     * @param L [type:lua_State*] lua state
     * @param buffer [type:dmScript::LuaHBuffer] buffer to push
     * @examples
     *
     * How to push a buffer and give Lua ownership of the buffer (GC)
     *
     * ```cpp
     * dmScript::LuaHBuffer luabuf = { buffer, dmScript::OWNER_LUA };
     * PushBuffer(L, luabuf);
     * ```
     *
     * How to push a buffer and keep ownership in C++
     *
     * ```cpp
     * dmScript::LuaHBuffer luabuf = { buffer, dmScript::OWNER_C };
     * PushBuffer(L, luabuf);
     * ```
     */
    void PushBuffer(lua_State* L, const LuaHBuffer& buffer);

    /*# retrieve a HBuffer from the supplied lua state
     *
     * Check if the value in the supplied index on the lua stack is a HBuffer and returns it.
     *
     * @name dmScript::CheckBuffer
     * @param L [type:lua_State*] lua state
     * @param index [type:int] Index of the value
     * @return buffer [type:LuaHBuffer*] pointer to dmScript::LuaHBuffer
     */
    LuaHBuffer* CheckBuffer(lua_State* L, int index);

    dmScript::LuaHBuffer* CheckBufferNoError(lua_State* L, int index);

    /*# get the value at index as a Vectormath::Aos::Vector3*
     * Get the value at index as a Vectormath::Aos::Vector3*
     * @name dmScript::ToVector3
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return v [type:Vectormath::Aos::Vector3*] The pointer to the value, or 0 if not correct type
     */
    Vectormath::Aos::Vector3* ToVector3(lua_State* L, int index);

    /*#
     * Check if the value at #index is a Vectormath::Aos::Vector3*
     * @name dmScript::IsVector3
     * @param L Lua state
     * @param index Index of the value
     * @return true if value at #index is a Vectormath::Aos::Vector3*
     */
    bool IsVector3(lua_State* L, int index);

    /*# push a Vectormath::Aos::Vector3 onto the Lua stack
     *
     * Push a Vectormath::Aos::Vector3 value onto the supplied lua state, will increase the stack by 1.
     * @name dmScript::PushVector3
     * @param L [type:lua_State*] Lua state
     * @param v [type:Vectormath::Aos::Vector3] Vector3 value to push
     */
    void PushVector3(lua_State* L, const Vectormath::Aos::Vector3& v);

    /*# check if the value is a Vectormath::Aos::Vector3
     *
     * Check if the value in the supplied index on the lua stack is a Vectormath::Aos::Vector3.
     * @note throws a luaL_error if it's not the correct type
     * @name dmScript::CheckVector3
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return vector3 [type:Vectormath::Aos::Vector3*] The pointer to the value
     */
    Vectormath::Aos::Vector3* CheckVector3(lua_State* L, int index);

    /*# get the value at index as a Vectormath::Aos::Vector4*
     * Get the value at index as a Vectormath::Aos::Vector4*
     * @name dmScript::ToVector4
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return v [type:Vectormath::Aos::Vector4*] The pointer to the value, or 0 if not correct type
     */
    Vectormath::Aos::Vector4* ToVector4(lua_State* L, int index);

    /*#
     * Check if the value at #index is a Vectormath::Aos::Vector4*
     * @name dmScript::IsVector4
     * @param L Lua state
     * @param index Index of the value
     * @return true if value at #index is a Vectormath::Aos::Vector4*
     */
    bool IsVector4(lua_State* L, int index);

    /*# push a Vectormath::Aos::Vector4 on the stack
     * Push a Vectormath::Aos::Vector4 value onto the supplied lua state, will increase the stack by 1.
     * @name dmScript::PushVector4
     * @param L [type:lua_State*] Lua state
     * @param v [type:Vectormath::Aos::Vector4] Vectormath::Aos::Vector4 value to push
     */
    void PushVector4(lua_State* L, const Vectormath::Aos::Vector4& v);

    /*# check if the value is a Vectormath::Aos::Vector3
     *
     * Check if the value in the supplied index on the lua stack is a Vectormath::Aos::Vector3.
     * @note throws a luaL_error if it's not the correct type
     * @name dmScript::CheckVector4
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return vector4 [type:Vectormath::Aos::Vector4*] The pointer to the value
     */
    Vectormath::Aos::Vector4* CheckVector4(lua_State* L, int index);

    /*# get the value at index as a Vectormath::Aos::Quat*
     * Get the value at index as a Vectormath::Aos::Quat*
     * @name dmScript::ToQuat
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return quat [type:Vectormath::Aos::Quat*] The pointer to the value, or 0 if not correct type
     */
    Vectormath::Aos::Quat* ToQuat(lua_State* L, int index);

    /*# push a Vectormath::Aos::Quat onto the Lua stack
     * Push a quaternion value onto Lua stack. Will increase the stack by 1.
     * @name dmScript::PushQuat
     * @param L [type:lua_State*] Lua state
     * @param quat [type:Vectormath::Aos::Quat] Vectormath::Aos::Quat value to push
     */
    void PushQuat(lua_State* L, const Vectormath::Aos::Quat& q);

    /*# check if the value is a Vectormath::Aos::Vector3
     *
     * Check if the value in the supplied index on the lua stack is a Vectormath::Aos::Quat.
     * @note throws a luaL_error if it's not the correct type
     * @name dmScript::CheckQuat
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return quat [type:Vectormath::Aos::Quat*] The pointer to the value
     */
    Vectormath::Aos::Quat* CheckQuat(lua_State* L, int index);

    /*# get the value at index as a Vectormath::Aos::Matrix4*
     * Get the value at index as a Vectormath::Aos::Matrix4*
     * @name dmScript::ToMatrix4
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return quat [type:Vectormath::Aos::Matrix4*] The pointer to the value, or 0 if not correct type
     */
    Vectormath::Aos::Matrix4* ToMatrix4(lua_State* L, int index);

    /*# push a Vectormath::Aos::Matrix4 onto the Lua stack
     * Push a matrix4 value onto the Lua stack. Will increase the stack by 1.
     * @name dmScript::PushMatrix4
     * @param L [type:lua_State*] Lua state
     * @param matrix [type:Vectormath::Aos::Matrix4] Vectormath::Aos::Matrix4 value to push
     */
    void PushMatrix4(lua_State* L, const Vectormath::Aos::Matrix4& m);

    /*# check if the value is a Vectormath::Aos::Matrix4
     *
     * Check if the value in the supplied index on the lua stack is a Vectormath::Aos::Matrix4.
     *
     * @note throws a luaL_error if it's not the correct type
     * @name dmScript::CheckMatrix4
     * @param L [type:lua_State*] Lua state
     * @param index [type:int] Index of the value
     * @return matrix [type:Vectormath::Aos::Matrix4*] The pointer to the value
     */
    Vectormath::Aos::Matrix4* CheckMatrix4(lua_State* L, int index);

    /*# convert a dmJson::Document to a Lua table
     * Convert a dmJson::Document document to Lua table.
     *
     * @name dmJson::Type
     * @param L [type:lua_State*] lua state
     * @param doc [type:dmJson::Document] JSON document
     * @param index [type:int] index of JSON node
     * @param error_str_out [type:char*] if an error is encountered, the error string is written to this argument
     * @param error_str_size [type:size_t] size of error_str_out
     * @return int [type:int] <0 if it fails. >=0 if it succeeds.
     */
    int JsonToLua(lua_State* L, dmJson::Document* doc, int index, char* error_str_out, size_t error_str_size);


    /*# callback info struct
     * callback info struct that will hold the relevant info needed to make a callback into Lua
     * @struct
     * @name dmScript::LuaCallbackInfo
     */
    struct LuaCallbackInfo;

    /*# Register a Lua callback.
     * Stores the current Lua state plus references to the script instance (self) and the callback.
     * Expects SetInstance() to have been called prior to using this method.
     *
     * The allocated data is created on the Lua stack and references are made against the
     * instances own context table.
     *
     * If the callback is not explicitly deleted with DestroyCallback() the references and
     * data will stay around until the script instance is deleted.
     *
     * @name dmScript::CreateCallback
     * @param L Lua state
     * @param index Lua stack index of the function
     * @return Lua callback struct if successful, 0 otherwise
     *
     * @examples
     *
     * ```cpp
     * static int SomeFunction(lua_State* L) // called from Lua
     * {
     *     LuaCallbackInfo* cbk = dmScript::CreateCallback(L, 1);
     *     ... store the callback for later
     * }
     *
     * static void InvokeCallback(LuaCallbackInfo* cbk)
     * {
     *     lua_State* L = dmScript::GetCallbackLuaContext(cbk);
     *     DM_LUA_STACK_CHECK(L, 0);
     *
     *     if (!dmScript::SetupCallback(callback))
     *     {
     *         return;
     *     }
     *
     *     lua_pushstring(L, "hello");
     *
     *     dmScript::PCall(L, 2, 0); // self + # user arguments
     *
     *     dmScript::TeardownCallback(callback);
     *     dmScript::DestroyCallback(cbk); // only do this if you're not using the callback again
     * }
     * ```
     */
    LuaCallbackInfo* CreateCallback(lua_State* L, int index);

    /*# Check if Lua callback is valid.
     * @name dmScript::IsCallbackValid
     * @param cbk Lua callback struct
     */
    bool IsCallbackValid(LuaCallbackInfo* cbk);

    /*# Deletes the Lua callback
     * @name dmScript::DestroyCallback
     * @param cbk Lua callback struct
     */
    void DestroyCallback(LuaCallbackInfo* cbk);

    /*# Gets the Lua context from a callback struct
     * @name dmScript::GetCallbackLuaContext
     * @param cbk Lua callback struct
     * @return L Lua state
     */
    lua_State* GetCallbackLuaContext(LuaCallbackInfo* cbk);


    /*# Setups up the Lua callback prior to a call to dmScript::PCall()
     *  The Lua stack after a successful call:
     * ```
     *    [-4] old instance
     *    [-3] context table
     *    [-2] callback
     *    [-1] self
     * ```
     *  In the event of an unsuccessful call, the Lua stack is unchanged
     *
     * @name dmScript::SetupCallback
     * @param cbk Lua callback struct
     * @return true if the setup was successful
     */
    bool SetupCallback(LuaCallbackInfo* cbk);

    /*# Cleans up the stack after SetupCallback+PCall calls
     * Sets the previous instance
     * Expects Lua stack:
     * ```
     *    [-2] old instance
     *    [-1] context table
     * ```
     * Both values are removed from the stack
     *
     * @name dmScript::TeardownCallback
     * @param cbk Lua callback struct
     */
    void TeardownCallback(LuaCallbackInfo* cbk);

    /*#
     * This function wraps lua_pcall with the addition of specifying an error handler which produces a backtrace.
     * In the case of an error, the error is logged and popped from the stack.
     *
     * @name dmScript::PCall
     * @param L lua state
     * @param nargs number of arguments
     * @param nresult number of results
     * @return error code from pcall
     */
    int PCall(lua_State* L, int nargs, int nresult);
}

#endif // DMSDK_SCRIPT_H
