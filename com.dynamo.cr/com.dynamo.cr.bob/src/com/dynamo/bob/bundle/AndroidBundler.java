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

package com.dynamo.bob.bundle;

import static org.apache.commons.io.FilenameUtils.normalize;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.charset.StandardCharsets;
import java.lang.StringBuilder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.io.IOUtils;

import com.dynamo.bob.Bob;
import com.dynamo.bob.CompileExceptionError;
import com.dynamo.bob.Platform;
import com.dynamo.bob.Project;
import com.dynamo.bob.fs.IResource;
import com.dynamo.bob.pipeline.ExtenderUtil;
import com.dynamo.bob.util.BobProjectProperties;
import com.dynamo.bob.util.Exec;
import com.dynamo.bob.util.Exec.Result;

import com.defold.extender.client.ExtenderResource;


public class AndroidBundler implements IBundler {
    private static Logger logger = Logger.getLogger(AndroidBundler.class.getName());

    private static Hashtable<Platform, String> platformToStripToolMap = new Hashtable<Platform, String>();
    static {
        platformToStripToolMap.put(Platform.Armv7Android, "strip_android");
        platformToStripToolMap.put(Platform.Arm64Android, "strip_android_aarch64");
    }

    private static Hashtable<Platform, String> platformToLibMap = new Hashtable<Platform, String>();
    static {
        platformToLibMap.put(Platform.Armv7Android, "armeabi-v7a");
        platformToLibMap.put(Platform.Arm64Android, "arm64-v8a");
    }

    private static void log(String s) { logger.log(Level.INFO, s); }

    private static void logResourceMap(Map<String, IResource> map) {
        for (String key : map.keySet()) {
            IResource value = map.get(key);
            log("key = " + key + " value = " + value);
        }
    }

    private static File createDir(File parent, String child) throws IOException {
        File dir = new File(parent, child);
        if (dir.exists()) {
            FileUtils.deleteDirectory(dir);
        }
        dir.mkdirs();
        return dir;
    }
    private static File createDir(String parent, String child) throws IOException {
        return createDir(new File(parent), child);
    }

    private static String readFile(String path) throws IOException {
        byte[] encoded = Files.readAllBytes(Paths.get(path));
        return new String(encoded, StandardCharsets.UTF_8);
    }

    private static void createFile(String path, String content) throws IOException {
        FileUtils.writeStringToFile(new File(path), content);
    }

    private static String getJavaBinFile(String file) {
        String javaHome = System.getProperty("java.home");
        if (javaHome == null) {
            javaHome = System.getenv("JAVA_HOME");
        }
        if (javaHome != null) {
            file = Paths.get(javaHome, "bin", file).toString();
        }
        return file;
    }

    private static Result exec(List<String> args) throws IOException {
        log("exec: " + String.join(" ", args));
        Map<String, String> env = new HashMap<String, String>();
        if (Platform.getHostPlatform() == Platform.X86_64Linux || Platform.getHostPlatform() == Platform.X86Linux) {
            env.put("LD_LIBRARY_PATH", Bob.getPath(String.format("%s/lib", Platform.getHostPlatform().getPair())));
        }
        return Exec.execResultWithEnvironment(env, args);
    }
    private static Result exec(String... args) throws IOException {
        return exec(Arrays.asList(args));
    }


    /**
    * Get keystore. If none is provided a debug keystore will be generated in the project root and used
    * when bundling.
    * https://stackoverflow.com/a/4055893/1266551
    */
    private static String getKeystore(Project project) throws IOException {
        String keystore = project.option("keystore", "");
        if (keystore.length() == 0) {
            keystore = "debug.keystore";
            String keystoreAlias = "androiddebugkey";
            String keystorePassword = "android";
            String keystorePasswordFile = "debug.keystore.pass.txt";
            if (!new File(keystore).exists()) {
                Result r = exec(getJavaBinFile("keytool"),
                    "-genkey",
                    "-v",
                    "-noprompt",
                    "-dname", "CN=Android, OU=Debug, O=Android, L=Unknown, ST=Unknown, C=US",
                    "-keystore", keystore,
                    "-storepass", keystorePassword,
                    "-alias", keystoreAlias,
                    "-keyalg", "RSA",
                    "-validity", "14000");

                if (r.ret != 0) {
                    throw new IOException(new String(r.stdOutErr));
                }
                createFile(keystorePasswordFile, keystorePassword);
            }

            project.setOption("keystore", keystore);
            project.setOption("keystore-pass", keystorePasswordFile);
            project.setOption("keystore-alias", keystoreAlias);
        }
        return keystore;
    }


    /**
     * Get keystore password. This loads the keystore password file and returns
     * the password stored in the file.
     */
    private static String getKeystorePassword(Project project) throws IOException, CompileExceptionError {
        String keystorePassword = project.option("keystore-pass", "");
        if (keystorePassword.length() == 0) {
            throw new CompileExceptionError(null, -1, "No keystore password");
        }
        return readFile(keystorePassword).trim();
    }

    /**
     * Get keystore alias. The keystore alias can either be a keystore alias
     * provided as a project option or the first alias in the keystore.
     */
    private static String getKeystoreAlias(Project project) throws IOException, CompileExceptionError {
        String keystore = getKeystore(project);
        String keystorePassword = getKeystorePassword(project);
        String alias = project.option("keystore-alias", "");
        if (alias.length() == 0) {

            List<String> args = new ArrayList<String>();
            args.add(getJavaBinFile("keytool"));
            args.add("-list");
            args.add("-keystore"); args.add(keystore);
            args.add("-storepass"); args.add(keystorePassword);
            args.add("-rfc");
            Result res = exec(args);
            if (res.ret != 0) {
                String msg = new String(res.stdOutErr);
                throw new IOException(msg);
            }

            Matcher m = Pattern.compile("Alias name: (.*)").matcher(new String(res.stdOutErr));
            if (!m.find()) {
                throw new CompileExceptionError(null, -1, "Unable to find keystore alias");
            }
            alias = m.group(1);
        }
        return alias;
    }

    private static String getProjectTitle(Project project) {
        BobProjectProperties projectProperties = project.getProjectProperties();
        String title = projectProperties.getStringValue("project", "title", "Unnamed");
        return title;
    }

    private static String getBinaryNameFromProject(Project project) {
        String title = getProjectTitle(project);
        String exeName = BundleHelper.projectNameToBinaryName(title);
        return exeName;
    }

    private static String getExtenderExeDir(Project project) {
        return FilenameUtils.concat(project.getRootDirectory(), "build");
    }

    private static List<Platform> getArchitectures(Project project) {
        return Platform.getArchitecturesFromString(project.option("architectures", ""), Platform.Armv7Android);
    }

    private static Platform getFirstPlatform(Project project) {
        return getArchitectures(project).get(0);
    }

    /**
    * Copy an engine binary to a destination file and optionally strip it of debug symbols
    */
    private static void copyEngineBinary(Project project, Platform architecture, File dest) throws IOException {
        // vanilla or extender exe?
        final String extenderExeDir = getExtenderExeDir(project);
        List<File> bundleExe = Bob.getNativeExtensionEngineBinaries(architecture, extenderExeDir);
        if (bundleExe == null) {
            final String variant = project.option("variant", Bob.VARIANT_RELEASE);
            bundleExe = Bob.getDefaultDmengineFiles(architecture, variant);
        }

        // copy the exe
        File exe = bundleExe.get(0);
        FileUtils.copyFile(exe, dest);

        // possibly also strip it
        final boolean strip_executable = project.hasOption("strip-executable");
        if (strip_executable) {
            String stripTool = Bob.getExe(Platform.getHostPlatform(), platformToStripToolMap.get(architecture));
            List<String> args = new ArrayList<String>();
            args.add(stripTool);
            args.add(dest.getAbsolutePath());
            Result res = exec(args);
            if (res.ret != 0) {
                throw new IOException(new String(res.stdOutErr));
            }
        }
    }

    /**
    * Get a list of dex files to include in the aab
    */
    private static ArrayList<File> getClassesDex(Project project) throws IOException {
        final String extenderExeDir = getExtenderExeDir(project);
        ArrayList<File> classesDex = new ArrayList<File>();

        for (Platform architecture : getArchitectures(project)) {
            List<File> bundleExe = Bob.getNativeExtensionEngineBinaries(architecture, extenderExeDir);
            if (bundleExe == null) {
                if (classesDex.isEmpty()) {
                    classesDex.add(new File(Bob.getPath("lib/classes.dex")));
                }
            }
            else {
                log("Using extender binary for architecture: " + architecture.toString());
                if (classesDex.isEmpty()) {
                    int i = 1;
                    while(true) {
                        String name = i == 1 ? "classes.dex" : String.format("classes%d.dex", i);
                        ++i;

                        File f = new File(FilenameUtils.concat(extenderExeDir, FilenameUtils.concat(architecture.getExtenderPair(), name)));
                        if (!f.exists())
                        break;
                        classesDex.add(f);
                    }
                }
            }
        }
        return classesDex;
    }

    private static List<String> trim(List<String> strings) {
        strings.replaceAll(String::trim);
        return strings;
    }

    private static void copyBundleResources(Project project, File outDir) throws CompileExceptionError {
        List<String> bundleResourcesPaths = trim(Arrays.asList(project.getProjectProperties().getStringValue("project", "bundle_resources", "").split(",")));
        List<String> bundleExcludeList = trim(Arrays.asList(project.getProjectProperties().getStringValue("project", "bundle_exclude_resources", "").split(",")));

        Set<String> set = new HashSet<>();
        set.addAll(Arrays.asList(Platform.Armv7Android.getExtenderPaths()));
        set.addAll(Arrays.asList(Platform.Arm64Android.getExtenderPaths()));
        List<String> platformFolders = new ArrayList<String>(set);

        for (String bundleResourcesPath : bundleResourcesPaths) {
            if (bundleResourcesPath.length() > 0) {
                for (String platformFolder : platformFolders) {
                    String platformPath = FilenameUtils.concat(bundleResourcesPath, platformFolder + "/res/");
                    if (ExtenderUtil.isAndroidAssetDirectory(project, platformPath)) {
                        Map<String, IResource> projectBundleResources = ExtenderUtil.collectResources(project, platformPath, bundleExcludeList);
                        ExtenderUtil.storeResources(outDir, projectBundleResources);
                    }
                }
            }
        }
    }

    /**
    * Copy Android resources per package
    */
    private static File copyResources(Project project, Platform platform, File outDir, BundleHelper helper, ICanceled canceled) throws IOException, CompileExceptionError {
        File androidResDir = createDir(outDir, "res");

        // Native extension build
        // Include all Android resources received from extender server
        File packagesDir = new File(project.getRootDirectory(), "build/"+platform.getExtenderPair()+"/packages");
        File packagesList = new File(packagesDir, "packages.txt");
        if(packagesList.exists()) {
            // create a list of resource directories, per package
            List<File> directories = new ArrayList<>();
            List<String> allLines = Files.readAllLines(packagesList.toPath());
            for (String line : allLines) {
                File resDir = new File(packagesDir, line);
                if (resDir.isDirectory()) {
                    directories.add(resDir);
                }
            }

            // copy all resources per package
            // packages/com.foo.bar/res/* -> res/com.foo.bar/*
            for (File src : directories) {
                Path srcPath = Path.of(src.getAbsolutePath());
                String packageName = srcPath.getName(srcPath.getNameCount() - 2).toString();
                File dest = createDir(androidResDir, packageName);
                for(File f : src.listFiles(File::isDirectory)) {
                    FileUtils.copyDirectoryToDirectory(f, dest);
                    BundleHelper.throwIfCanceled(canceled);
                }
            }
        }
        // Non-native extension build
        // Include local Android resources (icons and bundle resources)
        else {
            File resDir = new File(androidResDir, "com.defold.android");
            resDir.mkdirs();
            BundleHelper.createAndroidResourceFolders(resDir);
            helper.copyAndroidIcons(resDir);
            helper.copyAndroidPushIcons(resDir);

            File bundleResourceDestDir = new File(androidResDir, project.getProjectProperties().getStringValue("android", "package"));
            bundleResourceDestDir.mkdirs();
            copyBundleResources(project, bundleResourceDestDir);
        }


        return androidResDir;
    }

    /**
    * Compile android resources into "flat" files
    * https://developer.android.com/studio/build/building-cmdline#compile_and_link_your_apps_resources
    */
    private static File aapt2CompileResources(Project project, File androidResDir, ICanceled canceled) throws CompileExceptionError {
        log("Compiling resources from " + androidResDir.getAbsolutePath());
        try {
            // compile the resources using aapt2 to flat format files
            File compiledResourcesDir = Files.createTempDirectory("compiled_resources").toFile();
            compiledResourcesDir.deleteOnExit();

            // compile the resources for each package
            for (File packageDir : androidResDir.listFiles(File::isDirectory)) {
                File compiledResourceDir = createDir(compiledResourcesDir, packageDir.getName());

                List<String> args = new ArrayList<String>();
                args.add(Bob.getExe(Platform.getHostPlatform(), "aapt2"));
                args.add("compile");
                args.add("-o"); args.add(compiledResourceDir.getAbsolutePath());
                args.add("--dir"); args.add(packageDir.getAbsolutePath());

                Result res = exec(args);
                if (res.ret != 0) {
                    String msg = new String(res.stdOutErr);
                    throw new IOException(msg);
                }
                BundleHelper.throwIfCanceled(canceled);
            }

            return compiledResourcesDir;
        } catch (Exception e) {
            throw new CompileExceptionError(null, -1, "Failed compiling Android resources: " + e.getMessage());
        }
    }


    /**
    * Create apk from compiled resources and manifest file.
    * https://developer.android.com/studio/build/building-cmdline#compile_and_link_your_apps_resources
    */
    private static File aapt2LinkResources(Project project, File outDir, File compiledResourcesDir, File manifestFile, ICanceled canceled) throws CompileExceptionError {
        log("Linking resources from " + compiledResourcesDir.getAbsolutePath());
        try {
            File aabDir = new File(outDir, "aab");
            File apkDir = createDir(aabDir, "aapt2/apk");
            File outApk = new File(apkDir, "output.apk");

            List<String> args = new ArrayList<String>();
            args.add(Bob.getExe(Platform.getHostPlatform(), "aapt2"));
            args.add("link");
            args.add("--proto-format");
            args.add("-o"); args.add(outApk.getAbsolutePath());
            args.add("-I"); args.add(Bob.getPath("lib/android.jar"));
            args.add("--manifest"); args.add(manifestFile.getAbsolutePath());
            args.add("--auto-add-overlay");

            // write compiled resource list to a txt file
            StringBuilder sb = new StringBuilder();
            for (File resDir : compiledResourcesDir.listFiles(File::isDirectory)) {
                for (File file : resDir.listFiles()) {
                    if (file.getAbsolutePath().endsWith(".flat")) {
                        sb.append(file.getAbsolutePath() + " ");
                    }
                }
            }
            File resourceList = new File(aabDir, "compiled_resources.txt");
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(resourceList))) {
                writer.write(sb.toString());
            }
            args.add("-R"); args.add("@" + resourceList.getAbsolutePath());

            Result res = exec(args);
            if (res.ret != 0) {
                String msg = new String(res.stdOutErr);
                throw new IOException(msg);
            }
            BundleHelper.throwIfCanceled(canceled);
            return outApk;
        } catch (Exception e) {
            throw new CompileExceptionError(null, -1, "Failed building Android Application Bundle: " + e.getMessage());
        }
    }


    /**
    * Package pre-compiled code and resources
    * https://developer.android.com/studio/build/building-cmdline#package_pre-compiled_code_and_resources
    */
    private static File createAppBundleBaseZip(Project project, File outDir, File apk, ICanceled canceled) throws CompileExceptionError {
        log("Creating AAB base.zip");
        try {
            File aabDir = new File(outDir, "aab");

            // unzip the generated apk
            File apkUnzipDir = createDir(aabDir, "apk");
            BundleHelper.unzip(new FileInputStream(apk), apkUnzipDir.toPath());

            // create folder structure for the base.zip AAB module
            File baseDir = createDir(aabDir, "base");
            File dexDir = createDir(baseDir, "dex");
            File manifestDir = createDir(baseDir, "manifest");
            File assetsDir = createDir(baseDir, "assets");
            File libDir = createDir(baseDir, "lib");
            File resDir = createDir(baseDir, "res");

            FileUtils.copyFile(new File(apkUnzipDir, "AndroidManifest.xml"), new File(manifestDir, "AndroidManifest.xml"));
            FileUtils.copyFile(new File(apkUnzipDir, "resources.pb"), new File(baseDir, "resources.pb"));

            // copy classes.dex
            ArrayList<File> classesDex = getClassesDex(project);
            for (File classDex : classesDex) {
                log("Copying dex to " + classDex);
                FileUtils.copyFile(classDex, new File(dexDir, classDex.getName()));
                BundleHelper.throwIfCanceled(canceled);
            }

            // copy assets
            Map<String, IResource> bundleResources = ExtenderUtil.collectBundleResources(project, getArchitectures(project));
            ExtenderUtil.writeResourcesToDirectory(bundleResources, assetsDir);
            for (String name : Arrays.asList("game.projectc", "game.arci", "game.arcd", "game.dmanifest", "game.public.der")) {
                File source = new File(new File(project.getRootDirectory(), project.getBuildDirectory()), name);
                File dest = new File(assetsDir, name);
                log("Copying asset to " + dest);
                FileUtils.copyFile(source, dest);
                BundleHelper.throwIfCanceled(canceled);
            }

            // copy resources
            log("Copying resources to " + resDir);
            FileUtils.copyDirectory(new File(apkUnzipDir, "res"), resDir);

            // copy libs
            final String exeName = getBinaryNameFromProject(project);
            for (Platform architecture : getArchitectures(project)) {
                File architectureDir = createDir(libDir, platformToLibMap.get(architecture));
                File dest = new File(architectureDir, "lib" + exeName + ".so");
                log("Copying engine to " + dest);
                copyEngineBinary(project, architecture, dest);
                BundleHelper.throwIfCanceled(canceled);
            }

            // create base.zip
            File baseZip = new File(aabDir, "base.zip");
            log("Zipping " + baseDir + " to " + baseZip);
            if (baseZip.exists()) {
                baseZip.delete();
            }
            baseZip.createNewFile();
            ZipUtil.zipDirRecursive(baseDir, baseZip, canceled);
            return baseZip;
        } catch (Exception e) {
            throw new CompileExceptionError(null, -1, "Failed building Android Application Bundle: " + e.getMessage());
        }
    }

    /**
    * Build the app bundle using bundletool
    * https://developer.android.com/studio/build/building-cmdline#build_your_app_bundle_using_bundletool
    */
    private static File createBundle(Project project, File outDir, File baseZip, ICanceled canceled) throws CompileExceptionError {
        log("Creating Android Application Bundle");
        try {
            File bundletool = new File(Bob.getLibExecPath("bundletool-all.jar"));

            File baseAab = new File(outDir, getProjectTitle(project) + ".aab");

            List<String> args = new ArrayList<String>();
            args.add(getJavaBinFile("java")); args.add("-jar");
            args.add(bundletool.getAbsolutePath());
            args.add("build-bundle");
            args.add("--modules"); args.add(baseZip.getAbsolutePath());
            args.add("--output"); args.add(baseAab.getAbsolutePath());

            Result res = exec(args);
            if (res.ret != 0) {
                String msg = new String(res.stdOutErr);
                throw new IOException(msg);
            }
            BundleHelper.throwIfCanceled(canceled);
            return baseAab;
        } catch (Exception e) {
            throw new CompileExceptionError(null, -1, "Failed building Android Application Bundle: " + e.getMessage());
        }
    }

    /**
    * Sign file using jarsigner and keystore
    */
    private static void signFile(Project project, File signFile, ICanceled canceled) throws IOException, CompileExceptionError {
        log("Sign " + signFile);
        BundleHelper.throwIfCanceled(canceled);

        String keystore = getKeystore(project);
        String keystorePassword = getKeystorePassword(project);
        String keystoreAlias = getKeystoreAlias(project);

        Result r = exec("jarsigner",
            "-verbose",
            "-keystore", keystore,
            "-storepass", keystorePassword,
            signFile.getAbsolutePath(),
            keystoreAlias);
        if (r.ret != 0) {
            throw new IOException(new String(r.stdOutErr));
        }
    }

    /**
    * Copy debug symbols
    */
    private static void copySymbols(Project project, File outDir, ICanceled canceled) throws IOException, CompileExceptionError {
        final boolean has_symbols = project.hasOption("with-symbols");
        if (!has_symbols) {
            return;
        }
        File symbolsDir = new File(outDir, getProjectTitle(project) + ".apk.symbols");
        symbolsDir.mkdirs();
        final String exeName = getBinaryNameFromProject(project);
        final String extenderExeDir = getExtenderExeDir(project);
        final List<Platform> architectures = getArchitectures(project);
        final String variant = project.option("variant", Bob.VARIANT_RELEASE);
        for (Platform architecture : architectures) {
            List<File> bundleExe = Bob.getNativeExtensionEngineBinaries(architecture, extenderExeDir);
            if (bundleExe == null) {
                bundleExe = Bob.getDefaultDmengineFiles(architecture, variant);
            }
            File exe = bundleExe.get(0);
            File symbolExe = new File(symbolsDir, FilenameUtils.concat("lib/" + platformToLibMap.get(architecture), "lib" + exeName + ".so"));
            log("Copy debug symbols " + symbolExe);
            BundleHelper.throwIfCanceled(canceled);
            FileUtils.copyFile(exe, symbolExe);
        }

        BundleHelper.throwIfCanceled(canceled);

        File proguardMapping = new File(FilenameUtils.concat(extenderExeDir, FilenameUtils.concat(architectures.get(0).getExtenderPair(), "mapping.txt")));
        if (proguardMapping.exists()) {
            File symbolMapping = new File(symbolsDir, proguardMapping.getName());
            FileUtils.copyFile(proguardMapping, symbolMapping);
        }
    }

    /**
    * Cleanup bundle folder from intermediate folders and artifacts.
    */
    private static void cleanupBundleFolder(Project project, File outDir, File androidResDir, ICanceled canceled) throws IOException, CompileExceptionError {
        log("Cleanup bundle folder");
        BundleHelper.throwIfCanceled(canceled);
        FileUtils.deleteDirectory(androidResDir);
        FileUtils.deleteDirectory(new File(outDir, "aab"));
    }

    private static File createAAB(Project project, File outDir, BundleHelper helper, ICanceled canceled) throws IOException, CompileExceptionError {
        BundleHelper.throwIfCanceled(canceled);

        // STEP 1. copy android resources (icons, extension resources and manifest)
        // if bundling for both arm64 and armv7 we copy resources from one of the
        // architectures (it doesn't matter which one)
        final Platform platform = getFirstPlatform(project);
        File manifestFile = helper.copyOrWriteManifestFile(platform, outDir);
        File androidResDir = copyResources(project, platform, outDir, helper, canceled);

        // STEP 2. Use aapt2 to compile resources (to *.flat files)
        File compiledResDir = aapt2CompileResources(project, androidResDir, canceled);

        // STEP 3. Use aapt2 to create an APK containing resource files in protobuf format
        File apk = aapt2LinkResources(project, outDir, compiledResDir, manifestFile, canceled);

        // STEP 4. Extract protobuf files from the APK and create base.zip (manifest, assets, dex, res, lib, *.pb etc)
        File baseZip = createAppBundleBaseZip(project, outDir, apk, canceled);

        // STEP 5. Use bundletool to create AAB from base.zip
        File baseAab = createBundle(project, outDir, baseZip, canceled);

        //STEP 6. Sign AAB file
        signFile(project, baseAab, canceled);

        // STEP 7. Copy debug symbols
        copySymbols(project, outDir, canceled);

        // STEP 8. Cleanup bundle folder from intermediate folders and artifacts.
        cleanupBundleFolder(project, outDir, androidResDir, canceled);

        return baseAab;
    }



    /**
     * Create a universal APK set. A universal APK includes all of the app's
     * code and resources such that the APK is compatible with all device
     * configurations the app supports.
     */
    private static File createUniversalApks(Project project, File aab, File outDir, ICanceled canceled) throws IOException, CompileExceptionError {
        log("Creating universal APK set");
        String keystore = getKeystore(project);
        String keystorePassword = getKeystorePassword(project);
        String keystoreAlias = getKeystoreAlias(project);

        try {
            File bundletool = new File(Bob.getLibExecPath("bundletool-all.jar"));

            String aabPath = aab.getAbsolutePath();
            String name = FilenameUtils.getBaseName(aabPath);
            String apksPath = outDir.getAbsolutePath() + File.separator + name + ".apks";

            List<String> args = new ArrayList<String>();
            args.add(getJavaBinFile("java")); args.add("-jar");
            args.add(bundletool.getAbsolutePath());
            args.add("build-apks");
            args.add("--mode"); args.add("universal");
            args.add("--bundle"); args.add(aabPath);
            args.add("--output"); args.add(apksPath);
            args.add("--ks"); args.add(keystore);
            args.add("--ks-pass"); args.add("pass:" + keystorePassword);
            args.add("--ks-key-alias"); args.add(keystoreAlias);

            Result res = exec(args);
            if (res.ret != 0) {
                String msg = new String(res.stdOutErr);
                throw new IOException(msg);
            }
            BundleHelper.throwIfCanceled(canceled);
            return new File(apksPath);
        } catch (Exception e) {
            throw new CompileExceptionError(null, -1, "Failed building Android Application Bundle: " + e.getMessage());
        }
    }


    /**
     * Extract the universal APK from an APK set
     */
    private static File extractUniversalApk(File apks, File outDir, ICanceled canceled) throws IOException {
        log("Extracting universal APK from APK set");
        File apksDir = createDir(outDir, "apks");
        BundleHelper.unzip(new FileInputStream(apks), apksDir.toPath());

        File universalApk = new File(apksDir.getAbsolutePath() + File.separator + "universal.apk");
        File apk = new File(outDir.getAbsolutePath() + File.separator + FilenameUtils.getBaseName(apks.getPath()) + ".apk");
        universalApk.renameTo(apk);

        // cleanup
        FileUtils.deleteDirectory(apksDir);

        BundleHelper.throwIfCanceled(canceled);
        return apk;
    }


    private static File createAPK(File aab, Project project, File outDir, ICanceled canceled) throws IOException, CompileExceptionError {
        // STEP 1. Create universal APK set
        File apks = createUniversalApks(project, aab, outDir, canceled);

        // STEP 2. Extract universal.apk from APK set
        File apk = extractUniversalApk(apks, outDir, canceled);

        // cleanup
        apks.delete();

        return apk;
    }


    @Override
    public void bundleApplication(Project project, File bundleDir, ICanceled canceled) throws IOException, CompileExceptionError {
        Bob.initAndroid(); // extract resources

        final String variant = project.option("variant", Bob.VARIANT_RELEASE);
        BundleHelper helper = new BundleHelper(project, Platform.Armv7Android, bundleDir, variant);

        File outDir = createDir(bundleDir, getProjectTitle(project));

        String bundleFormat = project.option("bundle-format", "apk");
        if (bundleFormat.equals("aab")) {
            createAAB(project, outDir, helper, canceled);
        }
        else if (bundleFormat.equals("apk")) {
            File aab = createAAB(project, outDir, helper, canceled);
            File apk = createAPK(aab, project, outDir, canceled);
            aab.delete();
        }
        else {
            throw new CompileExceptionError(null, -1, "Unknown bundle format: " + bundleFormat);
        }
    }
}
