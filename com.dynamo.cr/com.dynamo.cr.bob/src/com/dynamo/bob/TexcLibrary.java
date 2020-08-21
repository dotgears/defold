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

package com.dynamo.bob;

import java.io.File;
import java.nio.Buffer;

import com.sun.jna.Native;
import com.sun.jna.Pointer;

public class TexcLibrary {

    static void addToPath(String variable, String path) {
        String newPath = null;

        // Check if jna.library.path is set externally.
        if (System.getProperty(variable) != null) {
            newPath = System.getProperty(variable);
        }

        if (newPath == null) {
            // Set path where texc_shared library is found.
            newPath = path;
        } else {
            // Append path where texc_shared library is found.
            newPath += File.pathSeparator + path;
        }

        // Set the concatenated jna.library path
        System.setProperty(variable, newPath);
        Bob.verbose("Set %s to '%s'", variable, newPath);
    }

    static {
        try {
            // Extract and append Bob bundled texc_shared path.
            Platform platform = Platform.getJavaPlatform();
            File lib = new File(Bob.getLib(platform, "texc_shared"));

            String libPath = lib.getParent();
            addToPath("jna.library.path", libPath);
            addToPath("java.library.path", libPath);

            String pvrTextLibPath = Bob.getLib(platform, "PVRTexLib"); // dependency of texc_shared

            // TODO: sad with a platform specific hack and placing dependency knowledge here but...
            if (platform == Platform.X86Linux || platform == Platform.X86_64Linux) {
                Bob.verbose("Loading PVRTexLib from:'%s'", pvrTextLibPath);
                System.load(pvrTextLibPath);
            }
            else if (platform == Platform.X86_64Win32 || platform == Platform.X86Win32) {
                final String msvcrlib = Bob.getLib(platform, "msvcr120"); // dependency of PVRTexLib
                // Make sure our version is loaded. Windows will do the lookup
                // for transitive dependencies, not jna, so we need to do this
                // explicitly or it will not be found.
                System.load(msvcrlib);
            }

            Native.register("texc_shared");
        } catch (Exception e) {
            System.out.println("FATAL: " + e.getMessage());
        }
    }

    public interface PixelFormat {
        public static int L8                = 0;
        public static int R8G8B8            = 1;
        public static int R8G8B8A8          = 2;
        public static int RGB_PVRTC_2BPPV1  = 3;
        public static int RGB_PVRTC_4BPPV1  = 4;
        public static int RGBA_PVRTC_2BPPV1 = 5;
        public static int RGBA_PVRTC_4BPPV1 = 6;
        public static int RGB_ETC1          = 7;
        public static int R5G6B5            = 8;
        public static int R4G4B4A4          = 9;
        public static int L8A8              = 10;

        /*
        JIRA issue: DEF-994
        public static int RGB_DXT1          = 8;
        public static int RGBA_DXT1         = 9;
        public static int RGBA_DXT3         = 10;
        public static int RGBA_DXT5         = 11;
        */
    }

    public interface ColorSpace {
        public static int LRGB = 0;
        public static int SRGB = 1;
    }

    public interface CompressionLevel {
        public static int CL_FAST    = 0;
        public static int CL_NORMAL  = 1;
        public static int CL_HIGH    = 2;
        public static int CL_BEST    = 3;
    }

    public interface CompressionType {
        public static int CT_DEFAULT    = 0;
        public static int CT_WEBP       = 1;
        public static int CT_WEBP_LOSSY = 2;
    }

    public enum FlipAxis {
        FLIP_AXIS_X(0),
        FLIP_AXIS_Y(1),
        FLIP_AXIS_Z(2);

        private final int value;
        FlipAxis(int value) { this.value = value; }
        public int getValue() { return this.value; }
    }

    public interface DitherType {
        public static int DT_NONE = 0;
        public static int DT_DEFAULT = 1;
    }

    public static native Pointer TEXC_Create(int width, int height, int pixelFormat, int colorSpace, Buffer data);
    public static native void TEXC_Destroy(Pointer texture);

    public static native int TEXC_GetDataSizeCompressed(Pointer texture, int minMap);
    public static native int TEXC_GetDataSizeUncompressed(Pointer texture, int minMap);
    public static native int TEXC_GetTotalDataSize(Pointer texture);
    public static native int TEXC_GetData(Pointer texture, Buffer outData, int maxOutDataSize);
    public static native int TEXC_GetCompressionFlags(Pointer texture);

    public static native boolean TEXC_Resize(Pointer texture, int width, int height);
    public static native boolean TEXC_PreMultiplyAlpha(Pointer texture);
    public static native boolean TEXC_GenMipMaps(Pointer texture);
    public static native boolean TEXC_Flip(Pointer texture, int flipAxis);
    public static native boolean TEXC_Transcode(Pointer texture, int pixelFormat, int colorSpace, int compressionLevel, int compressionType, int dither);


    public static native Pointer TEXC_CompressWebPBuffer(int width, int height, int bitsPerPixel, Buffer data, int datasize, int pixelFormat, int compressionLevel, int compressionType);
    public static native int TEXC_GetTotalBufferDataSize(Pointer buffer);
    public static native int TEXC_GetBufferData(Pointer buffer, Buffer outData, int maxOutDataSize);
    public static native void TEXC_DestroyBuffer(Pointer buffer);

}
