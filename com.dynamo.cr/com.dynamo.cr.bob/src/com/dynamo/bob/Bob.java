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

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.AccessDeniedException;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.FileSystemException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.ArrayList;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.apache.commons.cli.PosixParser;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.io.IOUtils;

import com.dynamo.bob.archive.EngineVersion;
import com.dynamo.bob.fs.DefaultFileSystem;
import com.dynamo.bob.fs.IResource;
import com.dynamo.bob.util.LibraryUtil;
import com.dynamo.bob.util.BobProjectProperties;

public class Bob {

    public static final String VARIANT_DEBUG = "debug";
    public static final String VARIANT_RELEASE = "release";
    public static final String VARIANT_HEADLESS = "headless";

    private static boolean verbose = false;
    private static File rootFolder = null;

    public Bob() {
    }

    // Registers a shutdown hook to delete the temp folder
    public static void registerShutdownHook() {
        final File tmpDirFile = rootFolder;

        // Add shutdown hook; creates a runnable that will recursively delete files in the temp directory.
        Runtime.getRuntime().addShutdownHook(new Thread(
          new Runnable() {
            @Override
            public void run() {
                try {
                    FileUtils.deleteDirectory(tmpDirFile);
                } catch (IOException e) {
                    // DE 20181012
                    // DEF-3533 Building with Bob causes exception when cleaning temp directory
                    // Failing to delete the files is not fatal, but not 100% clean.
                    // On Win32 we fail to delete dlls that are loaded since the OS locks them and this code runs before
                    // the dlls are unloaded.
                    // There is no explicit API to unload DLLs in Java/JNI, to accomplish this we need to do the
                    // class loading for the native functions differently and use a more indirect calling convention for
                    // com.defold.libs.TexcLibrary.
                    // See https://web.archive.org/web/20140704120535/http://www.codethesis.com/blog/unload-java-jni-dll
                    //
                    // For now we just issue a warning that we don't fully clean up.
                    System.out.println("Warning: Failed to clean up temp directory '" + tmpDirFile.getAbsolutePath() + "'");
                }
            }
        }));
      }

    private static void init() {
        if (rootFolder != null) {
            return;
        }

        try {
            String envRootFolder = System.getenv("DM_BOB_ROOTFOLDER");
            if (envRootFolder != null) {
                rootFolder = new File(envRootFolder);
                if (!rootFolder.exists()) {
                    rootFolder.mkdirs();
                }
                if (!rootFolder.isDirectory()) {
                    throw new IOException(String.format("Error when specifying DM_BOB_ROOTFOLDER: %s is not a directory!", rootFolder.getAbsolutePath()));
                }
                System.out.println("env DM_BOB_ROOTFOLDER=" + rootFolder);
            } else {
                rootFolder = Files.createTempDirectory(null).toFile();
                // Make sure we remove the temp folder on exit
                registerShutdownHook();
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public static void initLua() {
        init();
        try {
            extract(Bob.class.getResource("/lib/luajit-share.zip"), new File(rootFolder, "share"));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public static void initAndroid() {
        init();
        try {
            // Android SDK aapt is dynamically linked against libc++.so, we need to extract it so that
            // aapt will find it later when AndroidBundler is run.
            String libc_filename = Platform.getHostPlatform().getLibPrefix() + "c++" + Platform.getHostPlatform().getLibSuffix();
            URL libc_url = Bob.class.getResource("/lib/" + Platform.getHostPlatform().getPair() + "/" + libc_filename);
            if (libc_url != null) {
                File f = new File(rootFolder, Platform.getHostPlatform().getPair() + "/lib/" + libc_filename);
                atomicCopy(libc_url, f, false);
            }

            extract(Bob.class.getResource("/lib/android-res.zip"), rootFolder);

            // NOTE: android.jar and classes.dex aren't are only available in "full bob", i.e. from CI
            URL android_jar = Bob.class.getResource("/lib/android.jar");
            if (android_jar != null) {
                File f = new File(rootFolder, "lib/android.jar");
                atomicCopy(android_jar, f, false);
            }
            URL classes_dex = Bob.class.getResource("/lib/classes.dex");
            if (classes_dex != null) {
                File f = new File(rootFolder, "lib/classes.dex");
                atomicCopy(classes_dex, f, false);
            }

        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private static void extract(final URL url, File toFolder) throws IOException {

        ZipInputStream zipStream = new ZipInputStream(new BufferedInputStream(url.openStream()));

        try{
            ZipEntry entry = zipStream.getNextEntry();
            while (entry != null)
            {
                if (!entry.isDirectory()) {

                    File dstFile = new File(toFolder, entry.getName());
                    dstFile.deleteOnExit();
                    dstFile.getParentFile().mkdirs();

                    OutputStream fileStream = null;

                    try {
                        final byte[] buf;
                        int i;

                        fileStream = new FileOutputStream(dstFile);
                        buf = new byte[1024];
                        i = 0;

                        while((i = zipStream.read(buf)) != -1) {
                            fileStream.write(buf, 0, i);
                        }
                    } finally {
                        IOUtils.closeQuietly(fileStream);
                    }
                    verbose("Extracted '%s' from '%s' to '%s'", entry.getName(), url, dstFile.getAbsolutePath());
                }

                entry = zipStream.getNextEntry();
            }
        } finally {
            IOUtils.closeQuietly(zipStream);
        }
    }

    public static String getPath(String path) {
        init();
        File f = new File(rootFolder, path);
        if (!f.exists()) {
            throw new RuntimeException(String.format("location %s not found", f.toString()));
        }
        return f.getAbsolutePath();
    }

    public static List<String> getExes(Platform platform, String name) throws IOException {
        String[] exeSuffixes = platform.getExeSuffixes();
        List<String> exes = new ArrayList<String>();
        for (String exeSuffix : exeSuffixes) {
            exes.add(getExeWithExtension(platform, name, exeSuffix));
        }
        return exes;
    }

    public static String getExe(Platform platform, String name) throws IOException {
        List<String> exes = getExes(platform, name);
        if (exes.size() > 1) {
            throw new IOException("More than one alternative when getting binary executable for platform: " + platform.toString());
        }
        return exes.get(0);
    }

    // https://stackoverflow.com/a/30755071/468516
    private static final String ENOTEMPTY = "Directory not empty";
    private static void move(final File source, final File target) throws FileAlreadyExistsException, IOException {
        try {
            Files.move(source.toPath(), target.toPath(), StandardCopyOption.ATOMIC_MOVE);

        } catch (AccessDeniedException e) {
            // directory move collision on Windows
            throw new FileAlreadyExistsException(source.toString(), target.toString(), e.getMessage());

        } catch (FileSystemException e) {
            if (ENOTEMPTY.equals(e.getReason())) {
                // directory move collision on Unix
                throw new FileAlreadyExistsException(source.toString(), target.toString(), e.getMessage());
            } else {
                // other problem
                throw e;
            }
        }
    }

    private static void atomicCopy(URL source, File target, boolean executable) throws IOException {
        if (target.exists()) {
            return;
        }

        long t = System.nanoTime();
        File tmp = new File(target.getParent(), String.format("%s_%d", target.getName(), t));
        FileUtils.copyURLToFile(source, tmp);
        tmp.setExecutable(executable);

        try {
            move(tmp, target);
        } catch (FileAlreadyExistsException e) {
            // pass
            tmp.delete();
        }
    }

    public static String getExeWithExtension(Platform platform, String name, String extension) throws IOException {
        init();

        String exeName = platform.getPair() + "/" + platform.getExePrefix() + name + extension;
        File f = new File(rootFolder, exeName);
        if (!f.exists()) {
            URL url = Bob.class.getResource("/libexec/" + exeName);
            if (url == null) {
                throw new RuntimeException(String.format("/libexec/%s could not be found locally, create an application manifest to build the engine remotely.", exeName));
            }

            atomicCopy(url, f, true);
        }

        return f.getAbsolutePath();
    }

    public static String getLibExecPath(String filename) throws IOException {
        init();
        File f = new File(rootFolder, filename);
        if (!f.exists()) {
            URL url = Bob.class.getResource("/libexec/" + filename);
            if (url == null) {
                throw new RuntimeException(String.format("/libexec/%s not found", filename));
            }

            atomicCopy(url, f, false);
        }
        return f.getAbsolutePath();
    }

   public static String getDefaultDmengineExeName(String variant) {
        switch (variant)
        {
            case VARIANT_DEBUG:
                return "dmengine";
            case VARIANT_RELEASE:
                return "dmengine_release";
            case VARIANT_HEADLESS:
                return "dmengine_headless";
            default:
                throw new RuntimeException(String.format("Invalid variant %s", variant));
        }
    }

    public static List<String> getDefaultDmenginePaths(Platform platform, String variant) throws IOException {
        return getExes(platform, getDefaultDmengineExeName(variant));
    }

    public static List<File> getDefaultDmengineFiles(Platform platform, String variant) throws IOException {
        List<String> binaryPaths = getDefaultDmenginePaths(platform, variant);
        List<File> binaryFiles = new ArrayList<File>();
        for (String path : binaryPaths) {
            binaryFiles.add(new File(path));
        }
        return binaryFiles;
    }

    public static List<File> getNativeExtensionEngineBinaries(Platform platform, String extenderExeDir) throws IOException
    {
        List<String> binaryNames = platform.formatBinaryName("dmengine");
        List<File> binaryFiles = new ArrayList<File>();
        for (String binaryName : binaryNames) {
            File extenderExe = new File(FilenameUtils.concat(extenderExeDir, FilenameUtils.concat(platform.getExtenderPair(), binaryName)));

            // All binaries must exist, otherwise return null
            if (!extenderExe.exists()) {
                return null;
            }
            binaryFiles.add(extenderExe);
        }
        return binaryFiles;
    }

    public static String getLib(Platform platform, String name) throws IOException {
        init();

        String libName = platform.getPair() + "/" + platform.getLibPrefix() + name + platform.getLibSuffix();
        File f = new File(rootFolder, libName);
        if (!f.exists()) {
            URL url = Bob.class.getResource("/lib/" + libName);
            if (url == null) {
                throw new RuntimeException(String.format("/lib/%s not found", libName));
            }

            atomicCopy(url, f, true);
        }
        return f.getAbsolutePath();
    }

    private static CommandLine parse(String[] args) {
        Options options = new Options();
        options.addOption("r", "root", true, "Build root directory. Default is current directory");
        options.addOption("o", "output", true, "Output directory. Default is \"build/default\"");
        options.addOption("i", "input", true, "Source directory. Default is current directory");
        options.addOption("v", "verbose", false, "Verbose output");
        options.addOption("h", "help", false, "This help message");
        options.addOption("a", "archive", false, "Build archive");
        options.addOption("e", "email", true, "User email");
        options.addOption("u", "auth", true, "User auth token");

        options.addOption("p", "platform", true, "Platform (when bundling)");
        options.addOption("bo", "bundle-output", true, "Bundle output directory");
        options.addOption("bf", "bundle-format", true, "Format of the created bundle (Android: 'apk' and 'aab')");

        options.addOption("mp", "mobileprovisioning", true, "mobileprovisioning profile (iOS)");
        options.addOption(null, "identity", true, "Sign identity (iOS)");

        options.addOption("ce", "certificate", true, "DEPRECATED! Certificate (Android)");
        options.addOption("pk", "private-key", true, "DEPRECATED! Private key (Android)");

        options.addOption("ks", "keystore", true, "Deployment keystore used to sign APKs (Android)");
        options.addOption("ksp", "keystore-pass", true, "Pasword of the deployment keystore (Android)");
        options.addOption("ksa", "keystore-alias", true, "The alias of the signing key+cert you want to use (Android)");

        options.addOption("d", "debug", false, "Use debug version of dmengine (when bundling). Deprecated, use --variant instead");
        options.addOption(null, "variant", true, "Specify debug, release or headless version of dmengine (when bundling)");
        options.addOption(null, "strip-executable", false, "Strip the dmengine of debug symbols (when bundling iOS or Android)");
        options.addOption(null, "with-symbols", false, "Generate the symbol file (if applicable)");

        options.addOption("tp", "texture-profiles", true, "Use texture profiles (deprecated)");
        options.addOption("tc", "texture-compression", true, "Use texture compression as specified in texture profiles");
        options.addOption("k", "keep-unused", false, "Keep unused resources in archived output");

        options.addOption("br", "build-report", true, "Filepath where to save a build report as JSON");
        options.addOption("brhtml", "build-report-html", true, "Filepath where to save a build report as HTML");

        options.addOption(null, "build-server", true, "The build server (when using native extensions)");
        options.addOption(null, "defoldsdk", true, "What version of the defold sdk (sha1) to use");
        options.addOption(null, "binary-output", true, "Location where built engine binary will be placed. Default is \"<build-output>/<platform>/\"");

        options.addOption(null, "use-vanilla-lua", false, "Only ships vanilla source code (i.e. no byte code)");

        options.addOption("l", "liveupdate", true, "yes if liveupdate content should be published");

        options.addOption("ar", "architectures", true, "comma separated list of architectures to include for the platform");

        options.addOption(null, "settings", true, "a path to a game project settings file. more than one occurrance are allowed. the settings files are applied left to right.");

        options.addOption(null, "version", false, "Prints the version number to the output");

        // debug options
        options.addOption(null, "debug-ne-upload", false, "Outputs the files sent to build server as upload.zip");


        CommandLineParser parser = new PosixParser();
        CommandLine cmd = null;
        try {
            cmd = parser.parse(options, args);
        } catch (ParseException e) {
            System.err.println(e.getMessage());
            System.exit(5);
        }
        if (cmd.hasOption("h")) {
            HelpFormatter helpFormatter = new HelpFormatter( );
            helpFormatter.printHelp("bob [options] [commands]", options);
            System.exit(0);
        }
        if (cmd.hasOption("ce") || cmd.hasOption("pk")) {
            System.out.println("Android signing using certificate and private key is no longer supported. You must use keystore signing.");
            HelpFormatter helpFormatter = new HelpFormatter( );
            helpFormatter.printHelp("bob [options] [commands]", options);
            System.exit(1);
        }

        return cmd;
    }

    private static Project createProject(String rootDirectory, String buildDirectory, String email, String auth) {
        Project project = new Project(new DefaultFileSystem(), rootDirectory, buildDirectory);
        project.setOption("email", email);
        project.setOption("auth", auth);

        return project;
    }

    public static String logExceptionToString(int severity, IResource res, int line, String message)
    {
        String resourceString = "unspecified";
        if (res != null) {
            resourceString = res.toString();
        }
        String strSeverity = "ERROR";
        if (severity == MultipleCompileException.Info.SEVERITY_INFO)
            strSeverity = "INFO";
        else if (severity == MultipleCompileException.Info.SEVERITY_WARNING)
            strSeverity = "WARNING";
        return String.format("%s: %s:%d: '%s'\n", strSeverity, resourceString, line, message);
    }

    private static void setupProject(Project project, boolean resolveLibraries, String sourceDirectory) throws IOException, LibraryException, CompileExceptionError {
        ClassLoaderScanner scanner = new ClassLoaderScanner();
        project.scan(scanner, "com.dynamo.bob");
        project.scan(scanner, "com.dynamo.bob.pipeline");

        BobProjectProperties projectProperties = project.getProjectProperties();
        String dependencies = projectProperties.getStringValue("project", "dependencies", "");

        List<URL> libUrls = LibraryUtil.parseLibraryUrls(dependencies);
        project.setLibUrls(libUrls);
        if (resolveLibraries) {
            project.resolveLibUrls(new ConsoleProgress());
        }
        project.mount(new ClassLoaderResourceScanner());

        Set<String> skipDirs = new HashSet<String>(Arrays.asList(".git", project.getBuildDirectory(), ".internal"));
        project.findSources(sourceDirectory, skipDirs);
    }

    private static void mainInternal(String[] args) throws IOException, CompileExceptionError, URISyntaxException, LibraryException {
        System.setProperty("java.awt.headless", "true");
        System.setProperty("file.encoding", "UTF-8");
        String cwd = new File(".").getAbsolutePath();

        CommandLine cmd = parse(args);
        String buildDirectory = getOptionsValue(cmd, 'o', "build/default");
        String rootDirectory = getOptionsValue(cmd, 'r', cwd);
        String sourceDirectory = getOptionsValue(cmd, 'i', ".");
        verbose = cmd.hasOption('v');

        if (cmd.hasOption("version")) {
            System.out.println(String.format("bob.jar version: %s  sha1: %s  built: %s", EngineVersion.version, EngineVersion.sha1, EngineVersion.timestamp));
            System.exit(0);
            return;
        }

        if (cmd.hasOption("debug") && cmd.hasOption("variant")) {
            System.out.println("-d (--debug) option is deprecated and can't be set together with option --variant");
            System.exit(1);
            return;
        }

        if (cmd.hasOption("debug") && cmd.hasOption("strip-executable")) {
            System.out.println("-d (--debug) option is deprecated and can't be set together with option --strip-executable");
            System.exit(1);
            return;
        }

        String[] commands = cmd.getArgs();
        if (commands.length == 0) {
            commands = new String[] { "build" };
        }

        boolean shouldResolveLibs = false;
        for (String command : commands) {
            if (command.equals("resolve")) {
                shouldResolveLibs = true;
                break;
            }
        }

        String email = getOptionsValue(cmd, 'e', null);
        String auth = getOptionsValue(cmd, 'u', null);
        Project project = createProject(rootDirectory, buildDirectory, email, auth);

        if (cmd.hasOption("settings")) {
            for (String filepath : cmd.getOptionValues("settings")) {
                project.addPropertyFile(filepath);
            }
        }
        project.loadProjectFile();

        // resolves libraries and finds all sources
        setupProject(project, shouldResolveLibs, sourceDirectory);

        if (!cmd.hasOption("defoldsdk")) {
            project.setOption("defoldsdk", EngineVersion.sha1);
        }

        Option[] options = cmd.getOptions();
        for (Option o : options) {
            if (cmd.hasOption(o.getLongOpt())) {
                if (o.hasArg()) {
                    project.setOption(o.getLongOpt(), cmd.getOptionValue(o.getLongOpt()));
                } else {
                    project.setOption(o.getLongOpt(), "true");
                }
            }
        }

        // Get and set architectures list.
        Platform platform = project.getPlatform();
        String[] architectures = platform.getArchitectures().getDefaultArchitectures();
        List<String> availableArchitectures = Arrays.asList(platform.getArchitectures().getArchitectures());

        if (cmd.hasOption("architectures")) {
            architectures = cmd.getOptionValue("architectures").split(",");
        }

        if (architectures.length == 0) {
            System.out.println(String.format("ERROR! --architectures cannot be empty. Available architectures: %s", String.join(", ", availableArchitectures)));
            System.exit(1);
            return;
        }

        // Remove duplicates and make sure they are all supported for
        // selected platform.
        Set<String> uniqueArchitectures = new HashSet<String>();
        for (int i = 0; i < architectures.length; i++) {
            String architecture = architectures[i];
            if (!availableArchitectures.contains(architecture)) {
                System.out.println(String.format("ERROR! %s is not a supported architecture for %s platform. Available architectures: %s", architecture, platform.getPair(), String.join(", ", availableArchitectures)));
                System.exit(1);
                return;
            }
            uniqueArchitectures.add(architecture);
        }

        project.setOption("architectures", String.join(",", uniqueArchitectures));

        boolean shouldPublish = getOptionsValue(cmd, 'l', "no").equals("yes");
        project.setOption("liveupdate", shouldPublish ? "true" : "false");

        if (!cmd.hasOption("variant")) {
            if (cmd.hasOption("debug")) {
                System.out.println("WARNING option 'debug' is deprecated, use options 'variant' and 'strip-executable' instead.");
                project.setOption("variant", VARIANT_DEBUG);
            } else {
                project.setOption("variant", VARIANT_RELEASE);
                project.setOption("strip-executable", "true");
            }
        }

        String variant = project.option("variant", VARIANT_RELEASE);
        if (! (variant.equals(VARIANT_DEBUG) || variant.equals(VARIANT_RELEASE) || variant.equals(VARIANT_HEADLESS)) ) {
            System.out.println(String.format("--variant option must be one of %s, %s, or %s", VARIANT_DEBUG, VARIANT_RELEASE, VARIANT_HEADLESS));
            System.exit(1);
            return;
        }

        if (cmd.hasOption("texture-profiles")) {
            // If user tries to set (deprecated) texture-profiles, warn user and set texture-compression instead
            System.out.println("WARNING option 'texture-profiles' is deprecated, use option 'texture-compression' instead.");
            String texCompression = cmd.getOptionValue("texture-profiles");
            if (cmd.hasOption("texture-compression")) {
                texCompression = cmd.getOptionValue("texture-compression");
            }
            project.setOption("texture-compression", texCompression);
        }

        if (cmd.hasOption("use-vanilla-lua")) {
            project.setOption("use-vanilla-lua", "true");
        }

        if (cmd.hasOption("bundle-format")) {
            project.setOption("bundle-format", cmd.getOptionValue("bundle-format"));
        }

        boolean ret = true;
        StringBuilder errors = new StringBuilder();

        List<TaskResult> result = new ArrayList<>();
        try {
            result = project.build(new ConsoleProgress(), commands);
        } catch(MultipleCompileException e) {
            ret = false;
            errors.append("\n");
            for (MultipleCompileException.Info info : e.issues)
            {
                errors.append(logExceptionToString(info.getSeverity(), info.getResource(), info.getLineNumber(), info.getMessage()) + "\n");
            }
            errors.append("\nFull log: \n" + e.getRawLog() + "\n");
        }
        for (TaskResult taskResult : result) {
            if (!taskResult.isOk()) {
                ret = false;
                String message = taskResult.getMessage();
                if (message == null || message.isEmpty()) {
                    if (taskResult.getException() != null) {
                        message = taskResult.getException().getMessage();
                    } else {
                        message = "undefined";
                    }
                }
                errors.append(String.format("ERROR %s%s %s\n", taskResult.getTask().getInputs().get(0),
                        (taskResult.getLineNumber() != -1) ? String.format(":%d", taskResult.getLineNumber()) : "",
                        message));
                if (verbose) {
                    if (taskResult.getException() != null) {
                        errors.append("  ")
                                .append(taskResult.getException().toString())
                                .append("\n");
                        StackTraceElement[] elements = taskResult
                                .getException().getStackTrace();
                        for (StackTraceElement element : elements) {
                            errors.append("  ").append(element.toString())
                                    .append("\n");
                        }
                    }
                }
            }
        }
        if (!ret) {
            System.out.println("\nThe build failed for the following reasons:");
            System.out.println(errors.toString());
        }
        project.dispose();
        System.exit(ret ? 0 : 1);
    }

    public static void main(String[] args) throws IOException, CompileExceptionError, URISyntaxException, LibraryException {
        try {
            mainInternal(args);
        } catch (LibraryException e) {
            System.err.println(e.getMessage());
            System.err.println("Cause: " + e.getCause());
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static String getOptionsValue(CommandLine cmd, char o, String defaultValue) {
        String value = defaultValue;

        if (cmd.hasOption(o)) {
            value = cmd.getOptionValue(o);
        }
        return value;
    }

    public static void verbose(String message, Object... args) {
        if (verbose) {
            System.out.println("Bob: " + String.format(message, args));
        }
    }

}
