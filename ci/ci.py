#!/usr/bin/env python
# Copyright 2020 The Defold Foundation
# Licensed under the Defold License version 1.0 (the "License"); you may not use
# this file except in compliance with the License.
#
# You may obtain a copy of the License, together with FAQs at
# https://www.defold.com/license
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.



import sys
import subprocess
import platform
import os
import base64
from argparse import ArgumentParser

# The platforms we deploy our editor on
PLATFORMS_DESKTOP = ('x86_64-linux', 'x86_64-win32', 'x86_64-darwin')

def call(args, failonerror = True):
    print(args)
    process = subprocess.Popen(args, stdout = subprocess.PIPE, stderr = subprocess.STDOUT, shell = True)

    output = ''
    while True:
        line = process.stdout.readline()
        if line != '':
            output += line
            print(line.rstrip())
        else:
            break

    if process.wait() != 0 and failonerror:
        exit(1)

    return output


def platform_from_host():
    system = platform.system()
    if system == "Linux":
        return "x86_64-linux"
    elif system == "Darwin":
        return "x86_64-darwin"
    else:
        return "x86_64-win32"

def aptget(package):
    call("sudo apt-get install -y --no-install-recommends " + package)

def aptfast(package):
    call("sudo apt-fast install -y --no-install-recommends " + package)

def choco(package):
    call("choco install " + package + " -y")


def mingwget(package):
    call("mingw-get install " + package)


def setup_keychain(args):
    print("Setting up keychain")
    keychain_pass = "foobar"
    keychain_name = "defold.keychain"

    # create new keychain
    print("Creating keychain")
    # call("security delete-keychain {}".format(keychain_name))
    call("security create-keychain -p {} {}".format(keychain_pass, keychain_name))

    # set the new keychain as the default keychain
    print("Setting keychain as default")
    call("security default-keychain -s {}".format(keychain_name))

    # unlock the keychain
    print("Unlock keychain")
    call("security unlock-keychain -p {} {}".format(keychain_pass, keychain_name))

    # decode and import cert to keychain
    print("Decoding certificate")
    cert_path = os.path.join("ci", "cert.p12")
    cert_pass = args.keychain_cert_pass
    with open(cert_path, "wb") as file:
        file.write(base64.decodestring(args.keychain_cert))
    print("Importing certificate")
    # -A = allow access to the keychain without warning (https://stackoverflow.com/a/19550453)
    call("security import {} -k {} -P {} -A".format(cert_path, keychain_name, cert_pass))
    os.remove(cert_path)

    # required since macOS Sierra https://stackoverflow.com/a/40039594
    call("security set-key-partition-list -S apple-tool:,apple:,codesign: -s -k {} {}".format(keychain_pass, keychain_name))
    # prevent the keychain from auto-locking
    call("security set-keychain-settings {}".format(keychain_name))

    # add the keychain to the keychain search list
    call("security list-keychains -d user -s {}".format(keychain_name))

    print("Done with keychain setup")


def install(args):
    system = platform.system()
    print("Installing dependencies for system '%s' " % (system))
    if system == "Linux":
        # we use apt-fast to speed up apt-get downloads
        # https://github.com/ilikenwf/apt-fast
        call("sudo add-apt-repository ppa:apt-fast/stable")
        call("sudo apt-get update", failonerror=False)
        call("echo debconf apt-fast/maxdownloads string 16 | sudo debconf-set-selections")
        call("echo debconf apt-fast/dlflag boolean true | sudo debconf-set-selections")
        call("echo debconf apt-fast/aptmanager string apt-get | sudo debconf-set-selections")
        call("sudo apt-get install -y apt-fast aria2")

        call("sudo apt-get install -y software-properties-common")
        packages = [
            "libssl-dev",
            "openssl",
            "libtool",
            "autoconf",
            "automake",
            "build-essential",
            "uuid-dev",
            "libxi-dev",
            "libopenal-dev",
            "libgl1-mesa-dev",
            "libglw1-mesa-dev",
            "freeglut3-dev",
            "tofrodos",
            "tree",
            "valgrind",
            "lib32z1",
            "xvfb"
        ]
        aptfast(" ".join(packages))
    elif system == "Darwin":
        if args.keychain_cert:
            setup_keychain(args)


def build_engine(platform, with_valgrind = False, with_asan = False, with_vanilla_lua = False, skip_tests = False, skip_codesign = True, skip_docs = False, skip_builtins = False, archive = False, channel = None):
    args = 'python scripts/build.py distclean install_ext'.split()
    opts = []
    waf_opts = []

    opts.append('--platform=%s' % platform)

    if platform == 'js-web' or platform == 'wasm-web':
        args.append('install_ems')

    args.append('build_engine')

    if channel:
        opts.append('--channel=%s' % channel)

    if archive:
        args.append('archive_engine')

    if skip_codesign:
        opts.append('--skip-codesign')
    if skip_docs:
        opts.append('--skip-docs')
    if skip_builtins:
        opts.append('--skip-builtins')
    if skip_tests:
        opts.append('--skip-tests')
        waf_opts.append('--skip-build-tests')

    if with_valgrind:
        waf_opts.append('--with-valgrind')
    if with_asan:
        waf_opts.append('--with-asan')
    if with_vanilla_lua:
        waf_opts.append('--use-vanilla-lua')

    cmd = ' '.join(args + opts)

    # Add arguments to waf after a double-dash
    if waf_opts:
        cmd += ' -- ' + ' '.join(waf_opts)

    call(cmd)


def build_editor2(channel = None, engine_artifacts = None, skip_tests = False):
    host_platform = platform_from_host()
    if not host_platform in PLATFORMS_DESKTOP:
        return

    opts = []

    if engine_artifacts:
        opts.append('--engine-artifacts=%s' % engine_artifacts)

    if channel:
        opts.append('--channel=%s' % channel)

    if skip_tests:
        opts.append('--skip-tests')

    opts_string = ' '.join(opts)

    call('python scripts/build.py distclean install_ext build_editor2 --platform=%s %s' % (host_platform, opts_string))
    for platform in PLATFORMS_DESKTOP:
        call('python scripts/build.py bundle_editor2 --platform=%s %s' % (platform, opts_string))

def download_editor2(channel = None, platform = None):
    if platform is None:
        platforms = PLATFORMS_DESKTOP
    else:
        platforms = [platform]

    opts = []
    if channel:
        opts.append('--channel=%s' % channel)

    for platform in platforms:
        call('python scripts/build.py download_editor2 --platform=%s %s' % (platform, ' '.join(opts)))

def notarize_editor2(notarization_username = None, notarization_password = None, notarization_itc_provider = None):
    if not notarization_username or not notarization_password:
        print("No notarization username or password")
        exit(1)

    # args = 'python scripts/build.py download_editor2 notarize_editor2 archive_editor2'.split()
    args = 'python scripts/build.py notarize_editor2'.split()
    opts = []

    opts.append('--platform=x86_64-darwin')

    opts.append('--notarization-username=%s' % notarization_username)
    opts.append('--notarization-password=%s' % notarization_password)

    if notarization_itc_provider:
        opts.append('--notarization-itc-provider=%s' % notarization_itc_provider)

    cmd = ' '.join(args + opts)
    call(cmd)


def archive_editor2(channel = None, engine_artifacts = None, platform = None):
    if platform is None:
        platforms = PLATFORMS_DESKTOP
    else:
        platforms = [platform]

    opts = []
    if engine_artifacts:
        opts.append('--engine-artifacts=%s' % engine_artifacts)

    if channel:
        opts.append("--channel=%s" % channel)

    opts_string = ' '.join(opts)
    for platform in platforms:
        call('python scripts/build.py archive_editor2 --platform=%s %s' % (platform, opts_string))


def build_bob(channel = None):
    args = "python scripts/build.py distclean install_ext sync_archive build_bob archive_bob".split()
    opts = []

    if channel:
        opts.append("--channel=%s" % channel)

    cmd = ' '.join(args + opts)
    call(cmd)


def release(channel = None):
    args = "python scripts/build.py release".split()
    opts = []

    if channel:
        opts.append("--channel=%s" % channel)

    cmd = ' '.join(args + opts)
    call(cmd)


def build_sdk(channel = None):
    args = "python scripts/build.py build_sdk".split()
    opts = []

    if channel:
        opts.append("--channel=%s" % channel)

    cmd = ' '.join(args + opts)
    call(cmd)


def smoke_test():
    call('python scripts/build.py distclean install_ext smoke_test')


# https://stackoverflow.com/a/55276236/1266551
def get_branch():
    branch = call("git rev-parse --abbrev-ref HEAD").strip()
    if branch == "HEAD":
        branch = call("git rev-parse HEAD")
    return branch

def main(argv):
    parser = ArgumentParser()
    parser.add_argument('commands', nargs="+", help="The command to execute (engine, build-editor, notarize-editor, archive-editor, bob, sdk, install, smoke)")
    parser.add_argument("--platform", dest="platform", help="Platform to build for (when building the engine)")
    parser.add_argument("--with-asan", dest="with_asan", action='store_true', help="")
    parser.add_argument("--with-valgrind", dest="with_valgrind", action='store_true', help="")
    parser.add_argument("--with-vanilla-lua", dest="with_vanilla_lua", action='store_true', help="")
    parser.add_argument("--archive", dest="archive", action='store_true', help="Archive engine artifacts to S3")
    parser.add_argument("--skip-tests", dest="skip_tests", action='store_true', help="")
    parser.add_argument("--skip-builtins", dest="skip_builtins", action='store_true', help="")
    parser.add_argument("--skip-docs", dest="skip_docs", action='store_true', help="")
    parser.add_argument("--engine-artifacts", dest="engine_artifacts", help="Engine artifacts to include when building the editor")
    parser.add_argument("--keychain-cert", dest="keychain_cert", help="Base 64 encoded certificate to import to macOS keychain")
    parser.add_argument("--keychain-cert-pass", dest="keychain_cert_pass", help="Password for the certificate to import to macOS keychain")
    parser.add_argument('--notarization-username', dest='notarization_username', help="Username to use when sending the editor for notarization")
    parser.add_argument('--notarization-password', dest='notarization_password', help="Password to use when sending the editor for notarization")
    parser.add_argument('--notarization-itc-provider', dest='notarization_itc_provider', help="Optional iTunes Connect provider to use when sending the editor for notarization")

    args = parser.parse_args()

    platform = args.platform
    branch = get_branch()

    # configure build flags based on the branch
    release_channel = None
    skip_editor_tests = False
    if branch == "master":
        engine_channel = "stable"
        editor_channel = "editor-alpha"
        release_channel = "editor-stable"
        make_release = False
        engine_artifacts = args.engine_artifacts or "archived"
    elif branch == "beta":
        engine_channel = "beta"
        editor_channel = "beta"
        release_channel = engine_channel
        make_release = True
        engine_artifacts = args.engine_artifacts or "archived"
    elif branch == "dev":
        engine_channel = "alpha"
        editor_channel = "alpha"
        release_channel = engine_channel
        make_release = True
        engine_artifacts = args.engine_artifacts or "archived"
    elif branch == "editor-dev":
        engine_channel = None
        editor_channel = "editor-alpha"
        release_channel = editor_channel
        make_release = True
        engine_artifacts = args.engine_artifacts
    elif branch and branch.startswith("DEFEDIT-"):
        engine_channel = None
        editor_channel = None
        make_release = False
        engine_artifacts = args.engine_artifacts or "archived-stable"
    else: # engine dev branch
        engine_channel = "dev"
        editor_channel = None
        make_release = False
        skip_editor_tests = True
        engine_artifacts = args.engine_artifacts or "archived"

    print("Using branch={} engine_channel={} editor_channel={} engine_artifacts={}".format(branch, engine_channel, editor_channel, engine_artifacts))

    # execute commands
    for command in args.commands:
        if command == "engine":
            if not platform:
                raise Exception("No --platform specified.")
            build_engine(
                platform,
                with_valgrind = args.with_valgrind or (branch in [ "master", "beta" ]),
                with_asan = args.with_asan,
                with_vanilla_lua = args.with_vanilla_lua,
                archive = args.archive,
                skip_tests = args.skip_tests,
                skip_builtins = args.skip_builtins,
                skip_docs = args.skip_docs,
                channel = engine_channel)
        elif command == "build-editor":
            build_editor2(channel = editor_channel, engine_artifacts = engine_artifacts, skip_tests = skip_editor_tests)
        elif command == "download-editor":
            download_editor2(channel = editor_channel, platform = platform)
        elif command == "notarize-editor":
            notarize_editor2(
                notarization_username = args.notarization_username,
                notarization_password = args.notarization_password,
                notarization_itc_provider = args.notarization_itc_provider)
        elif command == "archive-editor":
            archive_editor2(channel = editor_channel, engine_artifacts = engine_artifacts, platform = platform)
        elif command == "bob":
            build_bob(channel = engine_channel)
        elif command == "sdk":
            build_sdk(channel = engine_channel)
        elif command == "smoke":
            smoke_test()
        elif command == "install":
            install(args)
        elif command == "release":
            if make_release:
                release(channel = release_channel)
            else:
                print("Branch '%s' is not configured for automatic release from CI" % branch)
        else:
            print("Unknown command {0}".format(command))


if __name__ == "__main__":
    main(sys.argv[1:])
