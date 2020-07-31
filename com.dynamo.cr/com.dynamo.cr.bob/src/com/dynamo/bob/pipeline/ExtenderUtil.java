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

package com.dynamo.bob.pipeline;

import static org.apache.commons.io.FilenameUtils.normalize;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.RegexFileFilter;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.error.YAMLException;

import com.defold.extender.client.ExtenderClient;
import com.defold.extender.client.ExtenderResource;

import com.dynamo.bob.CompileExceptionError;
import com.dynamo.bob.Platform;
import com.dynamo.bob.Project;
import com.dynamo.bob.fs.IResource;
import com.dynamo.bob.util.BobProjectProperties;

public class ExtenderUtil {

    public static final String appManifestPath = "_app/" + ExtenderClient.appManifestFilename;
    public static final String proguardPath = "_app/app.pro";

    private static class FSExtenderResource implements ExtenderResource {

        private IResource resource;
        FSExtenderResource(IResource resource) {
            this.resource = resource;
        }

        public IResource getResource() {
            return resource;
        }

        @Override
        public byte[] sha1() throws IOException {
            return resource.sha1();
        }

        @Override
        public String getAbsPath() {
            return resource.getAbsPath().replace('\\', '/');
        }

        @Override
        public String getPath() {
            return resource.getPath().replace('\\', '/');
        }

        @Override
        public byte[] getContent() throws IOException {
            return resource.getContent();
        }

        @Override
        public long getLastModified() {
            return resource.getLastModified();
        }

        @Override
        public String toString() {
            return resource.getPath().replace('\\', '/');
        }
    }

    public static class FileExtenderResource implements ExtenderResource {

        private File file;
        private String path;
        public FileExtenderResource(File file, String path) {
            this.file = file;
            this.path = path;
        }

        @Override
        public byte[] sha1() throws IOException {
            byte[] content = getContent();
            if (content == null) {
                throw new IllegalArgumentException(String.format("Resource '%s' is not created", getPath()));
            }
            MessageDigest sha1;
            try {
                sha1 = MessageDigest.getInstance("SHA1");
            } catch (NoSuchAlgorithmException e) {
                throw new RuntimeException(e);
            }
            sha1.update(content);
            return sha1.digest();
        }

        @Override
        public String getAbsPath() {
            return path;
        }

        @Override
        public String getPath() {
            return path;
        }

        @Override
        public byte[] getContent() throws IOException {
            File f = file;
            if (!f.exists())
                return null;

            byte[] buf = new byte[(int) f.length()];
            BufferedInputStream is = new BufferedInputStream(new FileInputStream(f));
            try {
                is.read(buf);
                return buf;
            } finally {
                is.close();
            }
        }

        @Override
        public long getLastModified() {
            return file.lastModified();
        }

        @Override
        public String toString() {
            return getPath();
        }

        public File getFile() {
            return file;
        }
    }

    private static class EmptyResource implements IResource {
    	private String rootDir;
    	private String path;

    	public EmptyResource(String rootDir, String path) {
            this.rootDir = rootDir;
            this.path = path;
        }

    	@Override
		public IResource changeExt(String ext) {
			return null;
		}

		@Override
		public byte[] getContent() throws IOException {
			return new byte[0];
		}

		@Override
		public void setContent(byte[] content) throws IOException {
		}

		@Override
		public byte[] sha1() throws IOException {
            byte[] content = getContent();
            if (content == null) {
                throw new IllegalArgumentException(String.format("Resource '%s' is not created", getPath()));
            }
            MessageDigest sha1;
            try {
                sha1 = MessageDigest.getInstance("SHA1");
            } catch (NoSuchAlgorithmException e) {
                throw new RuntimeException(e);
            }
            sha1.update(content);
            return sha1.digest();
		}

		@Override
		public boolean exists() {
			return true;
		}

		@Override
		public boolean isFile() {
			return false;
		}

		@Override
		public String getAbsPath() {
            return rootDir + "/" + path;
		}

		@Override
		public String getPath() {
			return path;
		}

		@Override
		public void remove() {
		}

		@Override
		public IResource getResource(String name) {
			return null;
		}

		@Override
		public IResource output() {
			return null;
		}

		@Override
		public boolean isOutput() {
			return false;
		}

		@Override
		public void setContent(InputStream stream) throws IOException {
		}

		@Override
		public long getLastModified() {
	        return new File(rootDir).lastModified();
		}

    }

    // Used to rename a resource in the multipart request and prefix the content with a base variant

    public static class FSAliasResource extends FSExtenderResource {
        private IResource resource;
        private String alias;
        private String rootDir;

        FSAliasResource(IResource resource, String rootDir, String alias) {
            super(resource);
            this.resource = resource;
            this.rootDir = rootDir;
            this.alias = alias;
        }

        public IResource getResource() {
            return resource;
        }

        @Override
        public byte[] sha1() throws IOException {
            byte[] content = getContent();
            if (content == null) {
                throw new IllegalArgumentException(String.format("Resource '%s' is not created", getPath()));
            }
            MessageDigest sha1;
            try {
                sha1 = MessageDigest.getInstance("SHA1");
            } catch (NoSuchAlgorithmException e) {
                throw new RuntimeException(e);
            }
            sha1.update(content);
            return sha1.digest();
        }

        @Override
        public String getAbsPath() {
            return rootDir + "/" + alias;
        }

        @Override
        public String getPath() {
            return alias;
        }

        @Override
        public byte[] getContent() throws IOException {
            byte[] content = resource.getContent();
            byte[] c = new byte[content.length];
            System.arraycopy(content, 0, c, 0, content.length);
            return c;
        }

        @Override
        public long getLastModified() {
            return resource.getLastModified();
        }

        @Override
        public String toString() {
            return getPath();
        }
    }

    public static class FSAppManifestResource extends FSAliasResource {
        private Map<String, String> options;

        FSAppManifestResource(IResource resource, String rootDir, String alias, Map<String, String> options) {
            super(resource, rootDir, alias);
            this.options = options;
        }

        @Override
        public byte[] getContent() throws IOException {
            String prefix = "";
            if (options != null) {
                prefix += "context:" + System.getProperty("line.separator");
                for (String key : options.keySet()) {
                    String value = options.get(key);
                    prefix += String.format("    %s: %s", key, value) + System.getProperty("line.separator");
                }
            }

            byte[] prefixBytes = prefix.getBytes();
            byte[] content = getResource().getContent();
            byte[] c = new byte[prefixBytes.length + content.length];
            System.arraycopy(prefixBytes, 0, c, 0, prefixBytes.length);
            System.arraycopy(content, 0, c, prefixBytes.length, content.length);
            return c;
        }
    }

    private static List<ExtenderResource> listFilesRecursive(Project project, String path) {
        List<ExtenderResource> resources = new ArrayList<ExtenderResource>();
        ArrayList<String> paths = new ArrayList<>();
        project.findResourcePaths(path, paths);
        for (String p : paths) {
            IResource r = project.getResource(p);
            // Note: findResourcePaths will return the supplied path even if it's not a file.
            // We need to check if the resource is not a directory before adding it to the list of paths found.
            if (r.isFile()) {
                resources.add(new FSExtenderResource(r));
            }
        }

        return resources;
    }

    public static List<ExtenderResource> listFilesRecursive(File baseDir, File subDir) {
        List<File> files = new ArrayList(FileUtils.listFiles(subDir, new RegexFileFilter(".*"), DirectoryFileFilter.DIRECTORY));

        List<ExtenderResource> resources = new ArrayList<>();
        for (File file : files) {
            String relative = baseDir.toURI().relativize(file.toURI()).getPath();
            resources.add(new FileExtenderResource(file, relative));
        }
        return resources;
    }

    private static List<String> trimExcludePaths(List<String> excludes) {
        List<String> trimmedExcludes = new ArrayList<String>();
        for (String path : excludes) {
            trimmedExcludes.add(path.trim());
        }
        return trimmedExcludes;
    }

    public static void mergeBundleMap(Map<String, IResource> into, Map<String, IResource> from, boolean allowOverrides) throws CompileExceptionError{

        Iterator<Map.Entry<String, IResource>> it = from.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, IResource> entry = (Map.Entry<String, IResource>)it.next();
            String outputPath = entry.getKey();
            if (!allowOverrides) {
                if (into.containsKey(outputPath)) {
                    IResource inputA = into.get(outputPath);
                    IResource inputB = entry.getValue();

                    String errMsg = "Conflicting output bundle resource '" + outputPath + "‘ generated by the following input files: " + inputA.toString() + " <-> " + inputB.toString();
                    throw new CompileExceptionError(inputB, 0, errMsg);
                }
            }
            into.put(outputPath, entry.getValue());
        }
    }

    /**
     * Get a list of paths to extension directories in the project.
     * @param project
     * @return A list of paths to extension directories
     */
    public static List<String> getExtensionFolders(Project project) {
        ArrayList<String> paths = new ArrayList<>();
        project.findResourcePaths("", paths);

        List<String> folders = new ArrayList<>();
        for (String p : paths) {
            File f = new File(p);
            if (f.getName().equals(ExtenderClient.extensionFilename)) {
                folders.add( f.getParent() );
            }
        }
        return folders;
    }

    private static boolean hasPropertyResource(Project project, BobProjectProperties projectProperties, String section, String key) {
        String path = projectProperties.getStringValue(section, key, "");
        if (!path.isEmpty()) {
            IResource resource = project.getResource(path);
            if (resource.exists()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns true if the project should build remotely
     * @param project
     * @return True if it contains native extension code
     */
    public static boolean hasNativeExtensions(Project project) {
        BobProjectProperties projectProperties = project.getProjectProperties();
        if (hasPropertyResource(project, projectProperties, "native_extension", "app_manifest") ||
            hasPropertyResource(project, projectProperties, "android", "proguard") &&
            !projectProperties.getStringValue("android", "proguard", "").startsWith("/builtins/")) {
            return true;
        }

        ArrayList<String> paths = new ArrayList<>();
        project.findResourcePaths("", paths);
        for (String p : paths) {
            File f = new File(p);
            if (f.getName().equals(ExtenderClient.extensionFilename)) {
                return true;
            }
        }
        return false;
    }

    private static IResource getPropertyResource(Project project, BobProjectProperties projectProperties, String section, String key) throws CompileExceptionError {
        String path = projectProperties.getStringValue(section, key, "");
        if (!path.isEmpty()) {
            IResource resource = project.getResource(path);
            if (resource.exists()) {
                return resource;
            } else {
                IResource projectResource = project.getResource("game.project");
                throw new CompileExceptionError(projectResource, 0, String.format("No such resource: %s.%s: %s", section, key, path));
            }
        }
        return null;
    }

    /**
     * Get a list of all extension sources and libraries from a project for a specific platform.
     * @param project
     * @return A list of IExtenderResources that can be supplied to ExtenderClient
     */
    public static List<ExtenderResource> getExtensionSources(Project project, Platform platform, Map<String, String> appmanifestOptions) throws CompileExceptionError {
        List<ExtenderResource> sources = new ArrayList<>();

        List<String> platformFolderAlternatives = new ArrayList<String>();
        platformFolderAlternatives.addAll(Arrays.asList(platform.getExtenderPaths()));
        platformFolderAlternatives.add("common");

        // Find app manifest if there is one
        BobProjectProperties projectProperties = project.getProjectProperties();
        {
            IResource resource = getPropertyResource(project, projectProperties, "native_extension", "app_manifest");
            if (resource == null) {
                 resource = new EmptyResource(project.getRootDirectory(), appManifestPath);
            }
            sources.add( new FSAppManifestResource(resource, project.getRootDirectory(), appManifestPath, appmanifestOptions ));
        }
        {
            IResource resource = getPropertyResource(project, projectProperties, "android", "proguard");
            if (resource != null) {
                sources.add(new FSAliasResource(resource, project.getRootDirectory(), proguardPath));
            }
        }

        // Find extension folders
        List<String> extensionFolders = getExtensionFolders(project);
        for (String extension : extensionFolders) {
            IResource resource = project.getResource(extension + "/" + ExtenderClient.extensionFilename);
            if (!resource.exists()) {
                throw new CompileExceptionError(resource, 1, "Resource doesn't exist!");
            }

            sources.add( new FSExtenderResource( resource ) );
            sources.addAll( listFilesRecursive( project, extension + "/include/" ) );
            sources.addAll( listFilesRecursive( project, extension + "/src/") );

            // Get "lib" and "manifest" folders; branches of into sub folders such as "common" and platform specifics
            for (String platformAlt : platformFolderAlternatives) {
                sources.addAll( listFilesRecursive( project, extension + "/lib/" + platformAlt + "/") );
                sources.addAll( listFilesRecursive( project, extension + "/manifests/" + platformAlt + "/") );
                sources.addAll( listFilesRecursive( project, extension + "/res/" + platformAlt + "/") );
            }
        }

        return sources;
    }

    /** Makes sure the project doesn't have duplicates wrt relative paths.
     * This is important since we use both files on disc and in memory. (DEF-3868, )
     * @return Doesn't return anything. It throws CompileExceptionError if the check fails.
     */
    public static void checkProjectForDuplicates(Project project) throws CompileExceptionError {
        Map<String, IResource> files = new HashMap<String, IResource>();

        ArrayList<String> paths = new ArrayList<>();
        project.findResourcePaths("", paths);
        for (String p : paths) {
            IResource r = project.getResource(p);
            if (!r.isFile()) // Skip directories
                continue;

            if (files.containsKey(r.getPath())) {
                IResource previous = files.get(r.getPath());
                throw new CompileExceptionError(r, 0, String.format("The files' relative path conflict:\n'%s' and\n'%s", r.getAbsPath(), r.getAbsPath()));
            }
            files.put(r.getPath(), r);
        }
    }

    /** Get the platform manifests from the extensions
     */
    public static List<IResource> getExtensionPlatformManifests(Project project, Platform platform, String name) throws CompileExceptionError {
        List<IResource> out = new ArrayList<>();

        List<String> platformFolderAlternatives = new ArrayList<String>();
        platformFolderAlternatives.addAll(Arrays.asList(platform.getExtenderPaths())); // we skip "common" here since it makes little sense

        // Find extension folders
        List<String> extensionFolders = getExtensionFolders(project);
        for (String extension : extensionFolders) {
            for (String platformAlt : platformFolderAlternatives) {
                List<ExtenderResource> files = listFilesRecursive( project, extension + "/manifests/" + platformAlt + "/");
                for (ExtenderResource r : files) {
                    if (!(r instanceof FSExtenderResource))
                        continue;
                    File f = new File(r.getAbsPath());
                    if (f.getName().equals(name)) {
                        out.add( ((FSExtenderResource)r).getResource() );
                    }
                }
            }
        }
        return out;
    }

    /**
     * Collect resources from a specific project path and a list of exclude paths.
     * @param project
     * @param platform String representing the target platform.
     * @param excludes A list of project relative paths for resources to exclude.
     * @return Returns a map with output paths as keys and the corresponding IResource that should be used as value.
     * @throws CompileExceptionError if a output conflict occurs.
     */
    public static Map<String, IResource> collectResources(Project project, String path, List<String> excludes) throws CompileExceptionError {
        if (excludes == null) {
            excludes = new ArrayList<>();
        }

        // Make sure the path has Unix separators, since this is how
        // paths are specified game project relative internally.
        path = FilenameUtils.separatorsToUnix(path);

        HashMap<String, IResource> resources = new HashMap<String, IResource>();
        ArrayList<String> paths = new ArrayList<>();
        project.findResourcePaths(path, paths);
        for (String p : paths) {
            String pathProjectAbsolute = "/" + p;
            if (!excludes.contains(pathProjectAbsolute)) {
                IResource r = project.getResource(p);
                // Note: findResourcePaths will return the supplied path even if it's not a file.
                // We need to check if the resource is not a directory before adding it to the list of paths found.
                if (r.isFile()) {
                    String bundleRelativePath = pathProjectAbsolute.substring(path.length());
                    resources.put(bundleRelativePath, r);
                }
            }
        }

        return resources;
    }
    private static String[] getBundleResourcePaths(Project project) {
        return project.getProjectProperties().getStringArrayValue("project", "bundle_resources", new String[0]);
    }

    // Removes resources that starts with the string
    // E.g. remove Android specific resources (aapt) from the list of bundle resources
    private static Map<String, IResource> pruneResourcesWithString(Map<String, IResource> resources, String pattern) {
        Map<String, IResource> tmp = new HashMap<>();
        for (String relativePath : resources.keySet()) {

            if (relativePath.startsWith(pattern)) {
                continue;
            }
            tmp.put(relativePath, resources.get(relativePath));
        }
        return tmp;
    }

    /**
     * Collect bundle resources based on a Project and a target platform string used to collect correct platform specific resources.
     * @param project
     * @param platform String representing the target platform.
     * @return Returns a map with output paths as keys and the corresponding IResource that should be used as value.
     * @throws CompileExceptionError if a output conflict occurs.
     */
    public static Map<String, IResource> collectBundleResources(Project project, List<Platform> platforms) throws CompileExceptionError {

        Map<String, IResource> bundleResources = new HashMap<String, IResource>();
        List<String> bundleExcludeList = trimExcludePaths(Arrays.asList(project.getProjectProperties().getStringValue("project", "bundle_exclude_resources", "").split(",")));
        List<String> platformFolderAlternatives = new ArrayList<String>();
        platformFolderAlternatives.add("common");
        for (Platform platform : platforms) {
            for (String platformAlt : platform.getExtenderPaths()) {        // e.g. "ios" + "armv7-ios"
                if (!platformFolderAlternatives.contains(platformAlt)) {
                    platformFolderAlternatives.add(platformAlt);
                }
            }
        }

        // Project specific bundle resources
        String[] bundleResourcesPaths = getBundleResourcePaths(project);
        for (String bundleResourcesPath : bundleResourcesPaths) {
            for (String platformAlt : platformFolderAlternatives) {
                String platformPath = FilenameUtils.concat(bundleResourcesPath, platformAlt + "/");
                Map<String, IResource> projectBundleResources = ExtenderUtil.collectResources(project, platformPath, bundleExcludeList);
                String platformResourcePath = "res/"; // the paths are relative to platformPath
                projectBundleResources = ExtenderUtil.pruneResourcesWithString(projectBundleResources, platformResourcePath);
                mergeBundleMap(bundleResources, projectBundleResources, true);
            }
        }

        // Get bundle resources from extensions
        List<String> extensionFolders = getExtensionFolders(project);
        for (String extension : extensionFolders) {
            for (String platformAlt : platformFolderAlternatives) {
                String platformPath = FilenameUtils.concat("/" + extension, "res/" + platformAlt + "/");
                Map<String, IResource> extensionBundleResources = ExtenderUtil.collectResources(project, platformPath, bundleExcludeList);
                String platformResourcePath = "res/"; // the paths are relative to platformPath
                extensionBundleResources = ExtenderUtil.pruneResourcesWithString(extensionBundleResources, platformResourcePath);
                mergeBundleMap(bundleResources, extensionBundleResources, true);
            }
        }

        return bundleResources;
    }


    public static boolean matchesAndroidAssetDirectoryName(String name) {
        // For the list of reserved names, see Table 1: https://developer.android.com/guide/topics/resources/providing-resources
        String[] assetDirs = new String[]{"values", "xml", "layout", "animator", "anim", "color", "drawable", "mipmap", "menu", "raw", "font"};
        for (String reserved : assetDirs) {
            if (name.startsWith(reserved)) {
                return true;
            }
        }
        return false;
    }

    // returns true if any sub directory has an android asset directory name
    public static boolean isAndroidAssetDirectory(Project project, String path) {
        List<String> subdirs = new ArrayList<>();

        // Make sure the path has Unix separators, since this is how
        // paths are specified game project relative internally.
        path = FilenameUtils.separatorsToUnix(path);
        project.findResourceDirs(path, subdirs);

        for (String subdir : subdirs) {
            String name = FilenameUtils.getName(FilenameUtils.normalizeNoEndSeparator(subdir));
            if (ExtenderUtil.matchesAndroidAssetDirectoryName(name)) {
                return true;
            }
        }
        return false;
    }

    // Collects all resources (even those inside the zip packages) and stores them into one single folder
    public static void storeResources(File targetDirectory, Map<String, IResource> resources) throws CompileExceptionError {
        for (String relativePath : resources.keySet()) {
            IResource r = resources.get(relativePath);
            File outputFile = new File(targetDirectory, relativePath);
            if (!outputFile.getParentFile().exists()) {
                outputFile.getParentFile().mkdirs();
            }
            try {
                byte[] data = r.getContent();
                FileUtils.writeByteArrayToFile(outputFile, data);
            } catch (Exception e) {
                throw new CompileExceptionError(r, 0, e);
            }
        }
    }

    private static Map<String, IResource> prependResourcePaths(Map<String, IResource> resources, String prefix) {
        Map<String, IResource> tmp = new HashMap<>();
        for (String relativePath : resources.keySet()) {
            // make sure the resources under /res/** doesn't collide between extensions by prepending the extension name
            String key = FilenameUtils.concat(prefix, Project.stripLeadingSlash(relativePath));
            tmp.put(key, resources.get(relativePath));
        }
        return tmp;
    }

    public static Map<String, IResource> getAndroidResources(Project project) throws CompileExceptionError {

        Map<String, IResource> androidResources = new HashMap<String, IResource>();
        List<String> bundleExcludeList = trimExcludePaths(Arrays.asList(project.getProjectProperties().getStringValue("project", "bundle_exclude_resources", "").split(",")));
        List<String> platformFolderAlternatives = new ArrayList<String>();

        List<String> armv7ExtenderPaths = new ArrayList<String>(Arrays.asList(Platform.Armv7Android.getExtenderPaths()));
        List<String> arm64ExtenderPaths = new ArrayList<String>(Arrays.asList(Platform.Arm64Android.getExtenderPaths()));
        Set<String> set = new LinkedHashSet<>(armv7ExtenderPaths);
        set.addAll(arm64ExtenderPaths);
        platformFolderAlternatives = new ArrayList<>(set);

        // Project specific bundle resources
        for (String bundleResourcesPath : getBundleResourcePaths(project)) {
            if (bundleResourcesPath.length() > 0) {
                for (String platformAlt : platformFolderAlternatives) {
                    String platformPath = FilenameUtils.concat(bundleResourcesPath, platformAlt + "/res/");
                    if (ExtenderUtil.isAndroidAssetDirectory(project, platformPath)) {
                        Map<String, IResource> projectBundleResources = ExtenderUtil.collectResources(project, platformPath, bundleExcludeList);
                        projectBundleResources = ExtenderUtil.prependResourcePaths(projectBundleResources, Project.stripLeadingSlash(bundleResourcesPath).replace('/', '_'));
                        mergeBundleMap(androidResources, projectBundleResources, false);
                    } else {
                        List<String> subdirs = new ArrayList<>();
                        project.findResourceDirs(platformPath, subdirs);
                        for (String subdir : subdirs) {
                            String subdirPath = FilenameUtils.concat(platformPath, subdir);
                            Map<String, IResource> projectBundleResources = ExtenderUtil.collectResources(project, subdirPath, bundleExcludeList);
                            projectBundleResources = ExtenderUtil.prependResourcePaths(projectBundleResources, Project.stripLeadingSlash(bundleResourcesPath + "/" + subdir).replace('/', '_'));
                            mergeBundleMap(androidResources, projectBundleResources, false);
                        }
                    }
                }
            }
        }

        // Get bundle resources from extensions
        List<String> extensionFolders = getExtensionFolders(project);
        for (String extension : extensionFolders) {
            for (String platformAlt : platformFolderAlternatives) {
                String platformPath = FilenameUtils.concat("/" + extension, "res/" + platformAlt + "/res/");
                if (ExtenderUtil.isAndroidAssetDirectory(project, platformPath)) {
                    Map<String, IResource> projectBundleResources = ExtenderUtil.collectResources(project, platformPath, bundleExcludeList);
                    projectBundleResources = ExtenderUtil.prependResourcePaths(projectBundleResources, extension);
                    mergeBundleMap(androidResources, projectBundleResources, false);

                } else {
                    List<String> subdirs = new ArrayList<>();
                    project.findResourceDirs(platformPath, subdirs);
                    for (String subdir : subdirs) {
                        String subdirPath = FilenameUtils.concat(platformPath, subdir);
                        Map<String, IResource> projectBundleResources = ExtenderUtil.collectResources(project, subdirPath, bundleExcludeList);
                        projectBundleResources = ExtenderUtil.prependResourcePaths(projectBundleResources, extension + "/" + subdir);
                        mergeBundleMap(androidResources, projectBundleResources, false);
                    }
                }
            }
        }

        return androidResources;
    }

    /**
     * Write a map of bundle resources to a specific disk directory.
     * @param resources Map of resources to write to disk.
     * @param directory File object pointing to a output directory.
     * @throws IOException
     */
    public static void writeResourcesToDirectory(Map<String, IResource> resources, File directory) throws IOException {
        Iterator<Map.Entry<String, IResource>> it = resources.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, IResource> entry = (Map.Entry<String, IResource>)it.next();
            File outputFile = new File(directory, entry.getKey());
            outputFile.getParentFile().mkdirs();
            FileUtils.writeByteArrayToFile(outputFile, entry.getValue().getContent());
        }
    }

    /**
     * Write a map of bundle resources to a Zip output stream.
     * @param resources Map of resources to write to disk.
     * @param zipOutputStream A ZipOutputStream where bundle resources should be written as Zip entries.
     * @throws IOException
     */
    public static void writeResourcesToZip(Map<String, IResource> resources, ZipOutputStream zipOutputStream) throws IOException {
        Iterator<Map.Entry<String, IResource>> it = resources.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, IResource> entry = (Map.Entry<String, IResource>)it.next();
            ZipEntry ze = new ZipEntry(normalize(entry.getKey(), true));
            zipOutputStream.putNextEntry(ze);
            zipOutputStream.write(entry.getValue().getContent());
        }
    }

    // For debugging purposes
    public static void writeResourcesToZip(List<ExtenderResource> source, ZipOutputStream zipOutputStream) throws IOException {
        for (ExtenderResource s : source) {
            ZipEntry ze = new ZipEntry(normalize(s.getPath(), true));
            zipOutputStream.putNextEntry(ze);
            zipOutputStream.write(s.getContent());
            zipOutputStream.closeEntry();
        }
    }

    /** Finds a resource given a relative path
     * @param path  The relative path to the resource
     * @param source A list of all source files
     * @return The resource, or null if not found
     */
    public static IResource getResource(String path, List<ExtenderResource> source) {
        for (ExtenderResource r : source) {
            if (r.getPath().equals(path)) {
                if (r instanceof ExtenderUtil.FSExtenderResource) {
                    ExtenderUtil.FSExtenderResource fsr = (ExtenderUtil.FSExtenderResource)r;
                    return fsr.getResource();
                } else {
                    // It was a generated file (e.g. R.java) which doesn't exist in the project
                    break;
                }
            }
        }
        return null;
    }

    private static int countLines(String str){
       String[] lines = str.split("\r\n|\r|\n");
       return lines.length;
    }

    public static Map<String, Object> readYaml(IResource resource) throws IOException {
        String yaml = new String(resource.getContent(), StandardCharsets.UTF_8);
        if (yaml.contains("\t")) {
            int numLines = 1 + countLines(yaml.substring(0, yaml.indexOf("\t")));
            throw new IOException(String.format("%s:%d: error: Manifest files are YAML files and cannot contain tabs. Indentation should be done with spaces.", resource.getAbsPath(), numLines));
        }

        try {
            return new Yaml().load(yaml);
        } catch(YAMLException e) {
            throw new IOException(String.format("%s:1: error: %s", resource.getAbsPath(), e.toString()));
        }
    }

    private static boolean isListOfStrings(Object l) {
        if (!(l instanceof List)) {
            return false;
        }
        List<Object> list = (List<Object>)l;
        for (Object o : list) {
            if (!(o instanceof String)) {
                return false;
            }
        }
        return true;
    }

    private static boolean isMap(Object o) {
        return o instanceof Map<?, ?>;
    }

    private static boolean isSameClass(Object a, Object b) {
        if (isMap(a)) return isMap(b);
        if (isListOfStrings(a)) return isListOfStrings(b);
        return a.getClass().equals(b.getClass());
    }

    /* Merges a and b
    Lists are merged: a + b = [*a, *b]
    Strings are not allowed, they will throw an error
    */
    @SuppressWarnings("unchecked")
    public static Map<String, Object> mergeManifestContext(Map<String, Object> a, Map<String, Object> b) throws RuntimeException {
        Map<String, Object> merged = new HashMap<String, Object>();
        Set<String> allKeys = new HashSet<String>();
        allKeys.addAll(a.keySet());
        allKeys.addAll(b.keySet());
        for (String key : allKeys) {
            Object objA = a.getOrDefault(key, null);
            Object objB = b.getOrDefault(key, null);
            if (objA == null || objB == null) {
                merged.put(key, objA == null ? objB : objA);
                continue;
            }

            if (!isSameClass(objA, objB)) {
                throw new RuntimeException(String.format("Class types differ: '%s' != '%s' for values '%s' and '%s'", objA.getClass(), objB.getClass(), objA, objB));
            }

            if (isListOfStrings(objA)) {
                List<String> listA = (List<String>)objA;
                List<String> listB = (List<String>)objB;
                List<String> list = new ArrayList<String>();
                list.addAll(listA);
                list.addAll(listB);
                merged.put(key, list);
            } else if (isMap(objA)) {
                Map<String, Object> mergedChildren = ExtenderUtil.mergeManifestContext((Map<String, Object>)objA, (Map<String, Object>)objB);
                merged.put(key, mergedChildren);
            } else {
                throw new RuntimeException(String.format("Unsupported value types: '%s' != '%s' for values '%s' and '%s'", objA.getClass(), objB.getClass(), objA, objB));
            }
        }
        return merged;
    }

    /*
    Reads all extension manifests and merges the platform context + children.
    An example of a manifest:

    name: foo
    platforms:
        armv7-android:
            context:
                ...
            bundle:
                ...
            ...


    In this case the values _under_ 'armv7-android' will be returned.
    */

    public static Map<String, Object> getPlatformSettingsFromExtensions(Project project, String platform) throws CompileExceptionError {
        List<String> folders = ExtenderUtil.getExtensionFolders(project);
        Map<String, Object> ctx = new HashMap<String, Object>();
        for (String folder : folders) {
            IResource resource = project.getResource(folder + "/" + ExtenderClient.extensionFilename);

            Map<String, Object> manifest = null;
            try {
                manifest = ExtenderUtil.readYaml(resource);
            } catch (Exception e) {
                throw new CompileExceptionError(resource, -1, e);
            }

            if (manifest == null) {
                throw new CompileExceptionError(resource, -1, "Could not parse extension manifest file.");
            }

            Map<String, Object> platforms = (Map<String, Object>)manifest.getOrDefault("platforms", null);
            if (platforms == null) {
                continue;
            }

            Map<String, Object> platform_ctx = (Map<String, Object>)platforms.getOrDefault(platform, null);
            if (platform_ctx == null) {
                continue;
            }

            try {
                ctx = mergeManifestContext(ctx, platform_ctx);
            } catch (RuntimeException e) {
                e.printStackTrace(System.out);
                throw new CompileExceptionError(resource, -1, String.format("Extension manifest '%s' contains invalid values: %s", resource.getAbsPath(), e.toString()));
            }
        }
        return ctx;
    }
}
