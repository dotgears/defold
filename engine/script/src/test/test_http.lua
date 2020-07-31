-- Copyright 2020 The Defold Foundation
-- Licensed under the Defold License version 1.0 (the "License"); you may not use
-- this file except in compliance with the License.
-- 
-- You may obtain a copy of the License, together with FAQs at
-- https://www.defold.com/license
-- 
-- Unless required by applicable law or agreed to in writing, software distributed
-- under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
-- CONDITIONS OF ANY KIND, either express or implied. See the License for the
-- specific language governing permissions and limitations under the License.

function callback(response)
end

requests_left = 7

function test_http()
    local headers = {}
    headers['X-A'] = 'Defold'
    headers['X-B'] = '!'
    http.request("http://127.0.0.1:" .. PORT, "GET",
        function(response)
            assert(response.status == 200)
            assert(response.response == "Hello Defold!")
            assert(response.headers.server == "Dynamo 1.0")
            requests_left = requests_left - 1
        end,
    headers)

    local post_data = "Some data to post..."
    http.request("http://127.0.0.1:" .. PORT, "POST",
        function(response)
            assert(response.status == 200)
            assert(response.response == "PONGSome data to post...")
            assert(response.headers.server == "Dynamo 1.0")
            requests_left = requests_left - 1
        end,
    headers, post_data)

    local binary_data = '\01\02\03'
    http.request("http://127.0.0.1:" .. PORT, "POST",
        function(response)
            assert(response.status == 200)
            assert(response.response == "PONG" .. binary_data)
            assert(response.headers.server == "Dynamo 1.0")
            requests_left = requests_left - 1
        end,
    headers, binary_data)

    local put_data = "Some data to put..."
    http.request("http://127.0.0.1:" .. PORT, "PUT",
        function(response)
            assert(response.status == 200)
            assert(response.response == "PONG_PUTSome data to put...")
            assert(response.headers.server == "Dynamo 1.0")
            requests_left = requests_left - 1
        end,
    headers, put_data)

    http.request("http://127.0.0.1:" .. PORT, "HEAD",
        function(response)
            assert(response.status == 200)
            assert(response.response == "")
            assert(response.headers.server == "Dynamo 1.0")
            assert(response.headers["content-length"] == "1234")
            requests_left = requests_left - 1
        end,
    headers)

    http.request("http://127.0.0.1:" .. PORT .. "/not_found", "GET",
        function(response)
            assert(response.status == 404)
            assert(response.response == "")
            assert(response.headers.server == "Dynamo 1.0")
            requests_left = requests_left - 1
        end,
    headers)

    http.request("http://foo.___", "GET",
        function(response)
            assert(response.status == 0)
            assert(response.response == "")
            assert(#response.response == 0)
            requests_left = requests_left - 1
        end,
    headers)


end

functions = { test_http = test_http }
