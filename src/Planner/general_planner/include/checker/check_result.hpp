#pragma once

#include <string>
#include <utility>

namespace general_planner::checker {
    enum class Severity {
        OK = 0,
        WARN,
        REJECT,
        FATAL
    };

    struct CheckResult {
        Severity severity{Severity::OK};
        std::string code{"OK"};
        std::string message;

        bool ok() const {
            return severity == Severity::OK || severity == Severity::WARN;
        }

        bool rejected() const {
            return severity == Severity::REJECT || severity == Severity::FATAL;
        }

        static CheckResult Ok() {
            return {};
        }

        static CheckResult Warn(std::string code_in, std::string message_in) {
            CheckResult result;
            result.severity = Severity::WARN;
            result.code = std::move(code_in);
            result.message = std::move(message_in);
            return result;
        }

        static CheckResult Reject(std::string code_in, std::string message_in) {
            CheckResult result;
            result.severity = Severity::REJECT;
            result.code = std::move(code_in);
            result.message = std::move(message_in);
            return result;
        }

        static CheckResult Fatal(std::string code_in, std::string message_in) {
            CheckResult result;
            result.severity = Severity::FATAL;
            result.code = std::move(code_in);
            result.message = std::move(message_in);
            return result;
        }
    };
}
