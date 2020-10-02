/* 
 * The automated RESTful build server that auto :
 *
 * - Rebuild dmEngine when requested
 * - Rebundle game version ( 1stly, HTML5 ) to local server
 *
 */


let express = require('express')
const { format } = require('path')

let app = express()

app.listen(7000, () => { console.log('Server running on port 7000') })


let BUILD_SERVER_IP = 'http://192.168.0.100:9000'
let DYNAMO_HOME = '/Users/thetrung/Documents/defold'
let INPUT_PATH = '../HTM5_saving/'
let OUTPUT_PATH = INPUT_PATH + 'build/'

let run_command = (cmd) => {
    const util = require('util')
    const exec = util.promisify(require('child_process').exec)
    let result = async () => { 
        try {
            console.log(`exec: ${cmd}`)
            const { stdout, stderr } = await exec(cmd)
            let summary = {
                'stdout' : stdout,
                'stderr' : stderr
            }
            return summary
        }catch (err) {
           console.error(err)
           return err
        }
    }
    return result()
} 
let shell_platform = (platform) => { run_command(`./scripts/build.py shell --platform=${platform} --package-path=./local_sdks/`) }

let bundle_web = () => { 
    return run_command(
        `(cd ${INPUT_PATH};java -jar ${DYNAMO_HOME}/tmp/dynamo_home/share/java/bob.jar --build-server ${BUILD_SERVER_IP} --archive --platform js-web distclean build bundle)`) 
}

let rebuild_engine_web = () => {
    return run_command(
        `sh cmd.sh --engine js-web;sh cmd.sh --engine wasm-web;sh cmd.sh --notify`)
}

//
// process GET request
//
app.get("/", (req, res, next) => {
    res.json(["Hello","Boss !","Please","ask","TrungB for further instruction !"])
})
//
// process bundle js-web request
//
app.get("/bundle_web", (req, res, next) => {
    (async () => {
        shell_platform('x86_64-darwin')
        let result = await bundle_web()
        await res.json([result])
    })()
})
//
// process rebuild js-web request
//
app.get("/rebuild_engine", (req, res, next) => {
    (async () => {
        shell_platform('x86_64-darwin')
        let result = await rebuild_engine_web()
        await res.json([result])
    })()
})