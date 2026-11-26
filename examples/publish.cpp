#include <chrono>
#include <iostream>
#include <string>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

int main()
{
    const std::string server_url = "http://127.0.0.1:8080/measurements";

    const double stamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch())
                             .count();

    const nlohmann::json measurement = {
        { "stamp", stamp },
        { "frame_id", "map" },
        { "client", "robot-1" },
        { "x", 366188.0 },
        { "y", 5609559.0 },
        { "yaw", 0.0 },
        { "sensor", "temperature" },
        { "unit", "C" },
        { "value", 21.5 }
    };
    const std::string body = measurement.dump();

    CURLcode result = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (result != CURLE_OK) {
        std::cerr << "Cannot initialize libcurl: "
                  << curl_easy_strerror(result) << '\n';
        return 1;
    }

    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        std::cerr << "Cannot create libcurl handle\n";
        curl_global_cleanup();
        return 1;
    }

    curl_slist* headers = curl_slist_append(nullptr, "Content-Type: application/json");
    if (headers == nullptr) {
        std::cerr << "Cannot allocate HTTP headers\n";
        curl_easy_cleanup(curl);
        curl_global_cleanup();
        return 1;
    }

    const auto set_option = [curl](CURLoption option, auto value) {
        const CURLcode code = curl_easy_setopt(curl, option, value);
        if (code != CURLE_OK) {
            std::cerr << "Cannot configure libcurl: "
                      << curl_easy_strerror(code) << '\n';
            return false;
        }
        return true;
    };

    const bool configured = set_option(CURLOPT_URL, server_url.c_str()) && set_option(CURLOPT_HTTPHEADER, headers) && set_option(CURLOPT_POSTFIELDS, body.c_str()) && set_option(CURLOPT_TIMEOUT, 5L);

    if (!configured) {
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
        curl_global_cleanup();
        return 1;
    }

    result = curl_easy_perform(curl);

    long status = 0;
    const CURLcode info_result = curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
    curl_global_cleanup();

    if (result != CURLE_OK) {
        std::cerr << "Request failed: " << curl_easy_strerror(result) << '\n';
        return 1;
    }
    if (info_result != CURLE_OK) {
        std::cerr << "Cannot read HTTP status: "
                  << curl_easy_strerror(info_result) << '\n';
        return 1;
    }

    std::cout << "\nHTTP status: " << status << '\n';
    return status >= 200 && status < 300 ? 0 : 1;
}