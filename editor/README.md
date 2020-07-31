# Defold Editor

## Requirements
* [Java 11](https://jdk.java.net/11/) ([installation instructions](#installing-jdk11) - Note that we do not yet officially support development with a newer version of Java than 11.
* Leiningen ([install instructions](#install-leiningen))

## Installing JDK 11
### macOS

* Download the [macOS](https://download.java.net/java/GA/jdk11/9/GPL/openjdk-11.0.2_osx-x64_bin.tar.gz) version and extract it
* Run `sudo cp -R <path-to-extracted-folder>/jdk-11.0.2.jdk /Library/Java/JavaVirtualMachines/`
* Verify that the jdk version is available by running `/usr/libexec/java_home -V`
* Switch to the new version by running `/usr/libexec/java_home -v 11.0.2`

### Linux

* Download [Linux](https://download.java.net/java/GA/jdk11/9/GPL/openjdk-11.0.2_linux-x64_bin.tar.gz) version and extract it somewhere
* Run `sudo update-alternatives --install "/usr/bin/java" "java" "/path/to/jdk/bin/java" 1102`
* Run `sudo update-alternatives --install "/usr/bin/javac" "javac" "/path/to/jdk/bin/javac" 1102`
* If require, switch to new version using `sudo update-alternatives --config java` and `sudo update-alternatives --config javac`

### Notes

If you are using IntelliJ for lein tasks, you will need to first add the new SDK (file->project structure/SDKs)
and then set the project SDK setting (file->project structure/Project) to the new version.

## Installing Leiningen
### Windows

First of all, follow the Windows instructions in [Defold Readme](../README.md)

* Start `msys.bat` as described
* Download the [lein.sh script](https://raw.githubusercontent.com/technomancy/leiningen/stable/bin/lein) from [Leiningen.org](http://leiningen.org) Put it somewhere in your (msys) path - if you're lazy, put it in `C:\MinGW\msys\1.0\bin`. You might need to `chmod a+x lein.sh`.
* Run `lein` in the `editor` subdirectory
  This will attempt to download leiningen and dependencies to your home directory.

  - If this fails with message

          Could not find or load main class clojure.main

    Try pointing your `HOME` environment variable to your “windows home”. For instance change it from `/home/Erik.Angelin` (msys) to `/c/Users/erik.angelin`:

        export HOME="/c/Users/erik.angelin"

    The problem seems to be that the (windows) java class path points to an invalid home directory.

  - If this fails because the github certificate cannot be verified:

          export HTTP_CLIENT='wget --no-check-certificate -O'

### macOS/Linux

* Install Leiningen `brew install leiningen`

## Setup
* Open a terminal/console and change directory to `defold`.

    `cd defold`

* Setup the shell environment (consider putting it in an alias in your bash profile):

    `./scripts/build.py shell --platform=...`

* Run install_ext for the correct platform:

    `./scripts/build.py install_ext --platform=...`

* Build the engine:

    `./scripts/build.py build_engine --platform=... --skip-tests -- --skip-build-tests`

* Build builtins:

    `scripts/build.py build_builtins`

* Build Bob:

    `scripts/build.py build_bob --skip-tests`

* Change directory to the editor directory

    `cd editor`

* Run:

    `lein init`

## Cursive IDE Setup
Some of the remaining instructions are about EMACS. If instead you want to use the Cursive IDE, read the [Cursive setup guide](README_CURSIVE.md).

## Running Tests
`lein test` will run all the tests including the integration tests.

If you are using a repl, you can also run the tests by calling `(suite/suite)`.

## Setup NREPL for debugging

If you want to work on the editor chances are you want to `connect` or `jack-in` to a REPL as described below.

For this to work you will need a `~/.lein/profiles.clj` file and put the nREPL, Cider (etc) dependencies there;

```
{:user {:plugins [[cider/cider-nrepl "0.10.2"]
                  [refactor-nrepl "1.1.0" :exclusions [org.clojure/clojure]]]
        :dependencies [[org.clojure/tools.nrepl "0.2.12"]]}}
```

Please note that Lein will introduce a nREPL dependency automagically, but its a good idea to override to your preferred version here anyway.

## Running the Editor
`lein run` will launch the editor as well as providing a nrepl port
for you to jack into

## Building the Editor

Use `scripts/bundle.py` to produce a bundled version of the editor.

There are a few different scenarios in which you might want to build the editor locally:

- Local editor sources, archived engine artifacts based on HEAD:
  - `./scripts/bundle.py --platform=x86_64-win32 --version=1.2.3.4 --engine-artifacts=archived`
    - This will fetch engine and launcher artifacts using the `HEAD` revision.
- Local editor sources, archived engine artifacts based on a different revision:
  - `./scripts/bundle.py --platform=x86_64-win32 --version=1.2.3.4 --engine-artifacts=archived-stable`
    - This will fetch engine and launcher artifacts using the latest stable revision and is handy if you are on a branch where no engine artifacts have been archived.
- Local editor sources, local engine artifacts, local launcher:
  - `./scripts/bundle.py --platform=x86_64-win32 --version=1.2.3.4 --engine-artifacts=dynamo-home`
    - This will use local engine artifacts from `$DYNAMO_HOME`, with the exception of the launcher.
- Local editor sources, archived engine artifacts, local launcher:
  - `./scripts/bundle.py --platform=x86_64-win32 --version=1.2.3.4  --engine-artifacts=archived --launcher ../tmp/dynamo_home/bin/x86_64-darwin/launcher`
    - This will fetch engine artifacts using the `HEAD` revision.


## Jacking into a REPL

You can also use `M-x cider-jack-in` or launch the editor inside Cursive for debugging with breakpoints etc.

First set the environment variable `DYNAMO_HOME`. Example of a value `/Users/martin/work/defold/tmp/dynamo_home`.

After you jacked in do the following to load and start the app;

```
user=> (dev)
dev=> (go)
```

## Running Benchmarks
`lein benchmark` will run the benchmarks and put the results to the
`test/benchmark/bench-result.txt` file. Make sure to have everything
on your system closed down

## Generating the docs
Running `lein doc` will generate the codox to the target/docs directory

## Styling
A single stylesheet is set on the root node (by convention) in the scene. The stylesheet `editor.css` is loaded as a regular java resource, from the uberjar or from the file-system in dev-mode. If an `editor.css` is found in the current working directory that file will take precedence over the aforementioned java resource.

The stylesheet can be reloaded with the function key `F5`.

The `editor.css` stylesheet is generated from the the sass/scss files in `styling/stylesheets`. To generate the file you can use either leiningen or gulp:

**leiningen**

- `lein sass once` to generate once
- `lein sass watch` to watch and re-generate css on changes

**nodejs**

In the `styling` directory:
- `npm install`

- `gulp` to generate once
- `gulp watch` to watch and re-generate css on changes

See `styling/README.md` for details.


### JavaFX Styling

The best way to understand how JavaFX styling works is by studying the default stylesheet `modena.css` included in `jfxrt.jar`

## Bundling games and running in browser

As a temporary solution, we use Bob (from Editor1) as the content pipeline for bundling and running in the browser. In order to setup Bob locally, you need to:

- Build the engine for the specific platform, e.g. `python scripts/build.py build_engine --platform=js-web --skip-tests -- --skip-build-tests`
  - For android, you also need to `build_go` through `build.py` to obtain `apkc`
- Build Bob with local artifacts, `python scripts/build.py build_bob`
- `lein init`, which will install `bob.jar` as a local maven package

## Using a local bob.jar

When developing it can be convenient to use a local bob.jar instead of the one bundled with the editor. This can by adding `,-Ddefold.dev=true` to the end of the `vmargs =` field of the editor config.

* On macOS the config is inside the application file: `Defold.app/Contents/Resources/config`
* On Windows and Linux the config file is located next to the Defold executable.
