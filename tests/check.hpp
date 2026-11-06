#pragma once
// A deliberately tiny test harness, so the project builds with nothing but a compiler.

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace check {

struct Test {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Test>& registry() {
    static std::vector<Test> tests;
    return tests;
}

inline int& failures() {
    static int n = 0;
    return n;
}

struct Register {
    Register(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline int run_all() {
    for (const auto& t : registry()) {
        int before = failures();
        t.fn();
        std::printf("%s %s\n", failures() == before ? "  ok  " : "  FAIL", t.name);
    }
    std::printf("\n%zu tests, %d failed checks\n", registry().size(), failures());
    return failures() == 0 ? 0 : 1;
}

}  // namespace check

#define CHECK_CONCAT_(a, b) a##b
#define CHECK_CONCAT(a, b) CHECK_CONCAT_(a, b)
#define TEST(name)                                                                  \
    static void name();                                                             \
    static check::Register CHECK_CONCAT(reg_, name)(#name, name);                   \
    static void name()

#define EXPECT(cond)                                                                       \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            ++check::failures();                                                           \
            std::printf("        %s:%d: expected %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                                  \
    } while (0)

#define EXPECT_EQ(a, b)                                                                              \
    do {                                                                                             \
        auto va_ = (a);                                                                              \
        auto vb_ = (b);                                                                              \
        if (!(va_ == vb_)) {                                                                         \
            ++check::failures();                                                                     \
            std::printf("        %s:%d: %s == %s (%s vs %s)\n", __FILE__, __LINE__, #a, #b,          \
                        std::to_string(va_).c_str(), std::to_string(vb_).c_str());                   \
        }                                                                                            \
    } while (0)
