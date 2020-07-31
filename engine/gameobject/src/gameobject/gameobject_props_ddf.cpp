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

#include "gameobject_props_ddf.h"

#include <stdlib.h>
#include <string.h>

namespace dmGameObject
{
    HPropertyContainer CreatePropertyContainerFromDDF(const dmPropertiesDDF::PropertyDeclarations* prop_descs)
    {
        PropertyContainerParameters params;

        params.m_NumberCount = prop_descs->m_NumberEntries.m_Count;
        params.m_HashCount = prop_descs->m_HashEntries.m_Count;
        params.m_URLStringCount = prop_descs->m_UrlEntries.m_Count;
        params.m_URLStringSize = 0;
        params.m_URLCount = 0;
        params.m_Vector3Count = prop_descs->m_Vector3Entries.m_Count;
        params.m_Vector4Count = prop_descs->m_Vector4Entries.m_Count;
        params.m_QuatCount = prop_descs->m_QuatEntries.m_Count;
        params.m_BoolCount = prop_descs->m_BoolEntries.m_Count;
        for (uint32_t i = 0; i < prop_descs->m_UrlEntries.m_Count; ++i)
        {
            params.m_URLStringSize += strlen(prop_descs->m_StringValues.m_Data[prop_descs->m_UrlEntries[i].m_Index]) + 1;
        }

        HPropertyContainerBuilder builder = CreatePropertyContainerBuilder(params);
        for (uint32_t i = 0; i < prop_descs->m_NumberEntries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_NumberEntries.m_Data[i];
            PushFloatType(builder, entry.m_Id, PROPERTY_TYPE_NUMBER, &prop_descs->m_FloatValues[entry.m_Index]);
        }
        for (uint32_t i = 0; i < prop_descs->m_Vector3Entries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_Vector3Entries.m_Data[i];
            PushFloatType(builder, entry.m_Id, PROPERTY_TYPE_VECTOR3, &prop_descs->m_FloatValues[entry.m_Index]);
        }
        for (uint32_t i = 0; i < prop_descs->m_Vector4Entries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_Vector4Entries.m_Data[i];
            PushFloatType(builder, entry.m_Id, PROPERTY_TYPE_VECTOR4, &prop_descs->m_FloatValues[entry.m_Index]);
        }
        for (uint32_t i = 0; i < prop_descs->m_QuatEntries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_QuatEntries.m_Data[i];
            PushFloatType(builder, entry.m_Id, PROPERTY_TYPE_QUAT, &prop_descs->m_FloatValues[entry.m_Index]);
        }
        for (uint32_t i = 0; i < prop_descs->m_BoolEntries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_BoolEntries.m_Data[i];
            PushBool(builder, entry.m_Id, prop_descs->m_FloatValues[entry.m_Index] != 0.0f);
        }
        for (uint32_t i = 0; i < prop_descs->m_HashEntries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_HashEntries.m_Data[i];
            PushHash(builder, entry.m_Id, prop_descs->m_HashValues[entry.m_Index]);
        }
        for (uint32_t i = 0; i < prop_descs->m_UrlEntries.m_Count; ++i)
        {
            const dmPropertiesDDF::PropertyDeclarationEntry& entry = prop_descs->m_UrlEntries.m_Data[i];
            PushURLString(builder, entry.m_Id, prop_descs->m_StringValues[entry.m_Index]);
        }
        return CreatePropertyContainer(builder);
    }

}
