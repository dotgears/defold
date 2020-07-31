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

#ifndef DMSDK_CONFIGFILE_H
#define DMSDK_CONFIGFILE_H

#include <stdint.h>

namespace dmConfigFile
{
    /*# SDK ConfigFile API documentation
     * [file:<dmsdk/dlib/configfile.h>]
     *
     * Configuration file access functions.
     * The configuration file is compiled version of the [file:game.project] file.
     *
     * @document
     * @name ConfigFile
     * @namespace dmConfigFile
     */

    /*# HConfig type definition
     *
     * ```cpp
     * typedef struct Config* HConfig;
     * ```
     *
     * @typedef
     * @name dmConfigFile::HConfig
     */
    typedef struct Config* HConfig;

    /*# get config value as string
     *
     * Get config value as string, returns default if the key isn't found
     *
     * @name dmConfigFile::GetString
     * @param config [type:dmBuffer::HConfig] Config file handle
     * @param key [type:const char*] Key in format section.key (.key for no section)
     * @param default_value [type:const char*] Default value to return if key isn't found
     * @return value [type:const char*] found value or default value
     */
    const char* GetString(HConfig config, const char* key, const char* default_value);

    /*# get config value as int
     *
     * Get config value as int, returns default if the key isn't found
     * Note: default_value is returned for invalid integer values
     *
     * @name dmConfigFile::GetInt
     * @param config [type:dmBuffer::HConfig] Config file handle
     * @param key [type:const char*] Key in format section.key (.key for no section)
     * @param default_value [type:int32_t] Default value to return if key isn't found
     * @return value [type:int32_t] found value or default value
     */
    int32_t GetInt(HConfig config, const char* key, int32_t default_value);

    /*# get config value as float
     *
     * Get config value as float, returns default if the key isn't found
     * Note: default_value is returned for invalid float values
     *
     * @name dmConfigFile::GetFloat
     * @param config [type:dmBuffer::HConfig] Config file handle
     * @param key [type:const char*] Key in format section.key (.key for no section)
     * @param default_value [type:float] Default value to return if key isn't found
     * @return value [type:float] found value or default value
     */
    float GetFloat(HConfig config, const char* key, float default_value);
}

#endif // DMSDK_CONFIGFILE_H

