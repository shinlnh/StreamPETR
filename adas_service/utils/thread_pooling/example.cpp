#include <ctpl.h>
#include <iostream>
#include <string>
#include "common.h"



void first(int id) {
    INFO("hello from: %d", id);
}

void aga(int id, int par) {
    INFO("hello from: %d, function with parameter %d", id, par);

}

struct Third {
    Third(int v) : v(v) {
        INFO("Third ctor %d", this->v);
    }
    Third(Third && c) : v(c.v) {
        INFO("Third move ctor");
    }
    Third(const Third & c) : v(c.v) {
        INFO("Third copy ctor");
    }
    ~Third() {
        INFO("Third dtor");
    }
    int v;
};

void mmm(int id, const std::string & s) {
    INFO("mmm function %d %s", id, s.c_str());
}

void ugu(int id, Third & t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    INFO("hello from %d, function with parameter Third %d", id, t.v);
}

int main(int argc, char **argv) {
    ctpl::thread_pool p(2 /* two threads in the pool */);

    std::future<void> qw = p.push(std::ref(first));  // function
    p.push(first);  // function
    p.push(aga, 7);  // function

    {
        struct Second {
            Second(const std::string & s) : s(s) {
                INFO("Second ctor");
            }
            Second(Second && c) : s(std::move(c.s)) {
                INFO("Second move ctor");
            }
            Second(const Second & c) : s(c.s) {
                INFO("Second copy ctor");
            }
            ~Second() {
                INFO("Second dtor");
            }
        
            void operator()(int id) const {
                INFO("hello from %d, %s", id, this->s.c_str());
            }
        private:
            std::string s;
        } second(", functor");

        p.push(std::ref(second));  // functor, reference
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        p.push(const_cast<const Second &>(second));  // functor, copy ctor
        p.push(std::move(second));  // functor, move ctor
        p.push(second);  // functor, move ctor
        p.push(Second(", functor"));  // functor, move ctor
    }
        {
            Third t(100);

            p.push(ugu, std::ref(t));  // function. reference
            p.push(ugu, t);  // function. copy ctor, move ctor
            p.push(ugu, std::move(t));  // function. move ctor, move ctor

        }
        p.push(ugu, Third(200));  // function



    std::string s = ", lambda";
    p.push([s](int id){  // lambda
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        INFO("hello from %d%s", id, s.c_str());
    });

    p.push([s](int id){  // lambda
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        INFO("hello from %d%s", id, s.c_str());
    });

    p.push(mmm, "worked");

    auto f = p.pop();
    if (f) {
        INFO("poped function from the pool");
        f(0);
    }
    // change the number of treads in the pool

    p.resize(1);

    std::string s2 = "result";
    auto f1 = p.push([s2](int){
        return s2;
    });
    // other code here
    //...
    INFO("returned %d", f1.get());

    auto f2 = p.push([](int){
        throw std::exception();
    });
    // other code here
    //...
    try {
        f2.get();
    }
    catch (std::exception & e) {
        INFO("caught exception");
    }

    // get thread 0
    auto & th = p.get_thread(0);

    return 0;
}
