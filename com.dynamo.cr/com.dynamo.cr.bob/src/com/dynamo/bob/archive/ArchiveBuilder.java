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

package com.dynamo.bob.archive;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.io.IOUtils;

import com.dynamo.bob.pipeline.ResourceNode;
import com.dynamo.crypt.Crypt;
import com.dynamo.liveupdate.proto.Manifest.HashAlgorithm;
import com.dynamo.liveupdate.proto.Manifest.SignAlgorithm;
import com.dynamo.liveupdate.proto.Manifest.ResourceEntryFlag;

import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4Factory;

public class ArchiveBuilder {

    public static final int VERSION = 4;
    public static final int HASH_MAX_LENGTH = 64; // 512 bits
    public static final int HASH_LENGTH = 20;
    public static final int MD5_HASH_DIGEST_BYTE_LENGTH = 16; // 128 bits

    private static final byte[] KEY = "aQj8CScgNP4VsfXK".getBytes();

    private static final List<String> ENCRYPTED_EXTS = Arrays.asList("luac", "scriptc", "gui_scriptc", "render_scriptc");

    private List<ArchiveEntry> entries = new ArrayList<ArchiveEntry>();
    private String root;
    private ManifestBuilder manifestBuilder = null;
    private LZ4Compressor lz4Compressor;
    private byte[] archiveIndexMD5 = new byte[MD5_HASH_DIGEST_BYTE_LENGTH];

    public ArchiveBuilder(String root, ManifestBuilder manifestBuilder) {
        this.root = new File(root).getAbsolutePath();
        this.manifestBuilder = manifestBuilder;
        this.lz4Compressor = LZ4Factory.fastestInstance().highCompressor();
    }

    private void add(String fileName, boolean doCompress, boolean isLiveUpdate) throws IOException {
        ArchiveEntry e = new ArchiveEntry(root, fileName, doCompress, isLiveUpdate);
        if (!contains(e)) {
            entries.add(e);
        }
    }

    public void add(String fileName, boolean doCompress) throws IOException {
        ArchiveEntry e = new ArchiveEntry(root, fileName, doCompress);
        if (!contains(e)) {
            entries.add(e);
        }
    }

    public void add(String fileName) throws IOException {
        ArchiveEntry e = new ArchiveEntry(root, fileName, false);
        if (!contains(e)) {
            entries.add(e);
        }
    }

    private boolean contains(ArchiveEntry e) {
        return entries.contains(e);
    }

    public ArchiveEntry getArchiveEntry(int index) {
        return this.entries.get(index);
    }

    public int getArchiveEntrySize() {
        return this.entries.size();
    }

    public byte[] getArchiveIndexHash() {
        return this.archiveIndexMD5;
    }

    public byte[] loadResourceData(String filepath) throws IOException {
        File fhandle = new File(filepath);
        return FileUtils.readFileToByteArray(fhandle);
    }

    public byte[] compressResourceData(byte[] buffer) {
        int maximumCompressedSize = lz4Compressor.maxCompressedLength(buffer.length);
        byte[] compressedContent = new byte[maximumCompressedSize];
        int compressedSize = lz4Compressor.compress(buffer, compressedContent);
        return Arrays.copyOfRange(compressedContent, 0, compressedSize);
    }

    public boolean shouldUseCompressedResourceData(byte[] original, byte[] compressed) {
        double ratio = (double) compressed.length / (double) original.length;
        return ratio <= 0.95;
    }

    public byte[] encryptResourceData(byte[] buffer) {
        return Crypt.encryptCTR(buffer, KEY);
    }

    public void writeResourcePack(String filename, String directory, byte[] buffer, byte flags, int size) throws IOException {
        FileOutputStream outputStream = null;
        try {
            File fhandle = new File(directory, filename);
            if (!fhandle.exists()) {
                byte[] padding = new byte[11];
                byte[] size_bytes = ByteBuffer.allocate(4).putInt(size).array();
                Arrays.fill(padding, (byte)0xED);
                outputStream = new FileOutputStream(fhandle);
                outputStream.write(size_bytes); // 4 bytes
                outputStream.write(flags); // 1 byte
                outputStream.write(padding); // 11 bytes
                outputStream.write(buffer);

            }
        } finally {
            IOUtils.closeQuietly(outputStream);
        }
    }

    // Checks if any of the parents are excluded
    // Parents are sorted, deepest parent first, root parent last
    public boolean isTreeExcluded(List<String> parents, List<String> excludedResources) {
        for (String parent : parents) {
            if (excludedResources.contains(parent))
                return true;
        }
        return false;
    }

    public boolean excludeResource(String filepath, List<String> excludedResources) {
        boolean result = false;
        if (this.manifestBuilder != null) {
            List<ArrayList<String>> parentChains = this.manifestBuilder.getParentCollections(filepath);
            for (List<String> parents : parentChains) {
                boolean excluded = isTreeExcluded(parents, excludedResources);
                if (!excluded) {
                    return false; // as long as one tree path requires this resource, we cannot exclude it
                }
                result = true;
            }
        }

        return result;
    }

    public void write(RandomAccessFile archiveIndex, RandomAccessFile archiveData, Path resourcePackDirectory, List<String> excludedResources) throws IOException {
        // INDEX
        archiveIndex.writeInt(VERSION); // Version
        archiveIndex.writeInt(0); // Pad
        archiveIndex.writeLong(0); // UserData, used in runtime to distinguish between if the index and resources are memory mapped or loaded from disk
        archiveIndex.writeInt(0); // EntryCount
        archiveIndex.writeInt(0); // EntryOffset
        archiveIndex.writeInt(0); // HashOffset
        archiveIndex.writeInt(0); // HashLength
        archiveIndex.write(new byte[MD5_HASH_DIGEST_BYTE_LENGTH]);

        int archiveIndexHeaderOffset = (int) archiveIndex.getFilePointer();

        Collections.sort(entries); // Since it has no hash, it sorts on path

        for (int i = entries.size() - 1; i >= 0; --i) {
            ArchiveEntry entry = entries.get(i);
            byte[] buffer = this.loadResourceData(entry.fileName);
            byte archiveEntryFlags = (byte) entry.flags;
            int resourceEntryFlags = ResourceEntryFlag.BUNDLED.getNumber();
            if (entry.compressedSize != ArchiveEntry.FLAG_UNCOMPRESSED) {
                // Compress data
                byte[] compressed = this.compressResourceData(buffer);
                if (this.shouldUseCompressedResourceData(buffer, compressed)) {
                    archiveEntryFlags = (byte)(archiveEntryFlags | ArchiveEntry.FLAG_COMPRESSED);
                    buffer = compressed;
                    entry.compressedSize = compressed.length;
                } else {
                    entry.compressedSize = ArchiveEntry.FLAG_UNCOMPRESSED;
                }
            }

            // Encrypt data
            String extension = FilenameUtils.getExtension(entry.fileName);
            if (ENCRYPTED_EXTS.indexOf(extension) != -1) {
                archiveEntryFlags = (byte) (archiveEntryFlags | ArchiveEntry.FLAG_ENCRYPTED);
                entry.flags = (entry.flags | ArchiveEntry.FLAG_ENCRYPTED);
                buffer = this.encryptResourceData(buffer);
            }

            // Add entry to manifest
            String normalisedPath = FilenameUtils.separatorsToUnix(entry.relName);

            // Calculate hash digest values for resource
            String hexDigest = null;
            try {
                byte[] hashDigest = ManifestBuilder.CryptographicOperations.hash(buffer, manifestBuilder.getResourceHashAlgorithm());
                entry.hash = new byte[HASH_MAX_LENGTH];
                System.arraycopy(hashDigest, 0, entry.hash, 0, hashDigest.length);
                hexDigest = ManifestBuilder.CryptographicOperations.hexdigest(hashDigest);
            } catch (NoSuchAlgorithmException exception) {
                throw new IOException("Unable to create a Resource Pack, the hashing algorithm is not supported!");
            }

            // Write resource to data archive
            if (this.excludeResource(normalisedPath, excludedResources)) {
                resourceEntryFlags = ResourceEntryFlag.EXCLUDED.getNumber();
                this.writeResourcePack(hexDigest, resourcePackDirectory.toString(), buffer, archiveEntryFlags, entry.size);
                entries.remove(i);
            } else {
                alignBuffer(archiveData, 4);
                entry.resourceOffset = (int) archiveData.getFilePointer();
                archiveData.write(buffer, 0, buffer.length);
            }

            manifestBuilder.addResourceEntry(normalisedPath, buffer, resourceEntryFlags);
        }

        Collections.sort(entries); // Since it has a hash, it sorts on hash

        // Write sorted hashes to index file
        int hashOffset = (int) archiveIndex.getFilePointer();
        for(ArchiveEntry entry : entries) {
            archiveIndex.write(entry.hash);
        }

        // Write sorted entries to index file
        int entryOffset = (int) archiveIndex.getFilePointer();
        alignBuffer(archiveIndex, 4);
        for (ArchiveEntry entry : entries) {
            archiveIndex.writeInt(entry.resourceOffset);
            archiveIndex.writeInt(entry.size);
            archiveIndex.writeInt(entry.compressedSize);
            archiveIndex.writeInt(entry.flags);
        }

        try {
            // Calc index file MD5 hash
            archiveIndex.seek(archiveIndexHeaderOffset);
            int num_bytes = (int) archiveIndex.length() - archiveIndexHeaderOffset;
            byte[] archiveIndexBytes = new byte[num_bytes];
            archiveIndex.readFully(archiveIndexBytes);
            this.archiveIndexMD5 = ManifestBuilder.CryptographicOperations.hash(archiveIndexBytes, HashAlgorithm.HASH_MD5);
        } catch (NoSuchAlgorithmException e) {
            System.err.println("The algorithm specified is not supported!");
            e.printStackTrace();
        }

        // Update index header with offsets
        archiveIndex.seek(0);
        archiveIndex.writeInt(VERSION);
        archiveIndex.writeInt(0); // Pad
        archiveIndex.writeLong(0); // UserData
        archiveIndex.writeInt(entries.size());
        archiveIndex.writeInt(entryOffset);
        archiveIndex.writeInt(hashOffset);
        archiveIndex.writeInt(ManifestBuilder.CryptographicOperations.getHashSize(manifestBuilder.getResourceHashAlgorithm()));
        archiveIndex.write(this.archiveIndexMD5);
    }

    private void alignBuffer(RandomAccessFile outFile, int align) throws IOException {
        int pos = (int) outFile.getFilePointer();
        int newPos = (int) (outFile.getFilePointer() + (align - 1));
        newPos &= ~(align - 1);

        for (int i = 0; i < (newPos - pos); ++i) {
            outFile.writeByte((byte) 0);
        }
    }

    private static void printUsageAndTerminate(String message) {
        System.err.println("Usage: ArchiveBuilder <root> <output> [-c] <file> [<file> ...]\n");
        System.err.println("  <root>            - directorypath to root of input files (<file>)");
        System.err.println("  <output>          - filepath for output content.");
        System.err.println("                      Three files (arci, arcd, dmanifest) will be generated.");
        System.err.println("  <file>            - filepath relative to <root> of file to build.");
        System.err.println("  -c                - Compress archive (default false).");
        if (message != null) {
            System.err.println("\nError: " + message);
        }

        System.exit(1);
    }

    public static void main(String[] args) throws IOException, NoSuchAlgorithmException {
        // Validate input
        if (args.length < 3) {
            printUsageAndTerminate("Too few arguments");
        }

        File dirpathRoot            = new File(args[0]);
        File filepathArchiveIndex   = new File(args[1] + ".arci");
        File filepathArchiveData    = new File(args[1] + ".arcd");
        File filepathManifest       = new File(args[1] + ".dmanifest");
        File filepathPublicKey      = new File(args[1] + ".public");
        File filepathPrivateKey     = new File(args[1] + ".private");
        File filepathManifestHash   = new File(args[1] + ".manifest_hash");

        if (!dirpathRoot.isDirectory()) {
            printUsageAndTerminate("root does not exist: " + dirpathRoot.getAbsolutePath());
        }

        boolean doCompress = false;
        boolean doOutputManifestHashFile = false;
        List<File> inputs = new ArrayList<File>();
        for (int i = 2; i < args.length; ++i) {
            if (args[i].equals("-c")) {
                doCompress = true;
            } else if (args[i].equals("-m")) {
                doOutputManifestHashFile = true;
            } else {
                File currentInput = new File(args[i]);
                if (!currentInput.isFile()) {
                    printUsageAndTerminate("file does not exist: " + currentInput.getAbsolutePath());
                }

                inputs.add(currentInput);
            }
        }

        if (inputs.isEmpty()) {
            printUsageAndTerminate("There must be at least one file");
        }

        // Create manifest and archive

        ManifestBuilder manifestBuilder = new ManifestBuilder(doOutputManifestHashFile);
        manifestBuilder.setProjectIdentifier("<anonymous project>");
        manifestBuilder.addSupportedEngineVersion(EngineVersion.sha1);
        manifestBuilder.setResourceHashAlgorithm(HashAlgorithm.HASH_SHA1);
        manifestBuilder.setSignatureHashAlgorithm(HashAlgorithm.HASH_SHA256);
        manifestBuilder.setSignatureSignAlgorithm(SignAlgorithm.SIGN_RSA);

        System.out.println("Generating private key: " + filepathPrivateKey.getCanonicalPath());
        System.out.println("Generating public key: " + filepathPublicKey.getCanonicalPath());
        ManifestBuilder.CryptographicOperations.generateKeyPair(SignAlgorithm.SIGN_RSA, filepathPrivateKey.getAbsolutePath(), filepathPublicKey.getAbsolutePath());
        manifestBuilder.setPrivateKeyFilepath(filepathPrivateKey.getAbsolutePath());
        manifestBuilder.setPublicKeyFilepath(filepathPublicKey.getAbsolutePath());

        ResourceNode rootNode = new ResourceNode("<AnonymousRoot>", "<AnonymousRoot>");

        int archivedEntries = 0;
        int excludedEntries = 0;
        ArchiveBuilder archiveBuilder = new ArchiveBuilder(dirpathRoot.toString(), manifestBuilder);
        for (File currentInput : inputs) {
            if (currentInput.getName().startsWith("liveupdate.")){
                excludedEntries++;
                archiveBuilder.add(currentInput.getAbsolutePath(), doCompress, true);
            } else {
                archivedEntries++;
                archiveBuilder.add(currentInput.getAbsolutePath(), doCompress);
            }
            ResourceNode currentNode = new ResourceNode(currentInput.getPath(), currentInput.getAbsolutePath());
            rootNode.addChild(currentNode);
        }
        System.out.println("Added " + Integer.toString(archivedEntries + excludedEntries) + " entries to archive (" + Integer.toString(excludedEntries) + " entries tagged as 'liveupdate' in archive).");

        manifestBuilder.setDependencies(rootNode);

        RandomAccessFile archiveIndex = new RandomAccessFile(filepathArchiveIndex, "rw");
        RandomAccessFile archiveData  = new RandomAccessFile(filepathArchiveData, "rw");
        archiveIndex.setLength(0);
        archiveData.setLength(0);

        Path resourcePackDirectory = Files.createTempDirectory("tmp.defold.resourcepack_");
        FileOutputStream outputStreamManifest = new FileOutputStream(filepathManifest);
        try {
            System.out.println("Writing " + filepathArchiveIndex.getCanonicalPath());
            System.out.println("Writing " + filepathArchiveData.getCanonicalPath());

            List<String> excludedResources = new ArrayList<String>();
            archiveBuilder.write(archiveIndex, archiveData, resourcePackDirectory, excludedResources);
            manifestBuilder.setArchiveIdentifier(archiveBuilder.getArchiveIndexHash());

            System.out.println("Writing " + filepathManifest.getCanonicalPath());
            byte[] manifestFile = manifestBuilder.buildManifest();
            outputStreamManifest.write(manifestFile);

            if (doOutputManifestHashFile) {
                if (filepathManifestHash.exists()) {
                    filepathManifestHash.delete();
                    filepathManifestHash.createNewFile();
                }
                FileOutputStream manifestHashOutoutStream = new FileOutputStream(filepathManifestHash);
                manifestHashOutoutStream.write(manifestBuilder.getManifestDataHash());
                manifestHashOutoutStream.close();
            }
        } finally {
            FileUtils.deleteDirectory(resourcePackDirectory.toFile());
            try {
                archiveIndex.close();
                archiveData.close();
                outputStreamManifest.close();
            } catch (IOException exception) {
                // Nothing to do, moving on ...
            }
        }

        System.out.println("Done.");
    }
}
