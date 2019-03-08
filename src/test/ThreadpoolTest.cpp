/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 2/27/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#define BOOST_TEST_MODULE threadpool_test

#include <iostream>
#include <thread>
#include <chrono>

#include <boost/test/included/unit_test.hpp>
#include "../tools/ThreadPool.h"

int calculate(int a, int b)
{
    std::cout << "calculate" << std::endl;
    int result = a + b;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    return result;
}

void do_some_thing()
{
    std::cout << "do_some_thing" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

BOOST_AUTO_TEST_SUITE(threadpool_test) // name of the test suite

    BOOST_AUTO_TEST_CASE(test1)
    {
        ThreadPool pool;
    }

    BOOST_AUTO_TEST_CASE(test2)
    {
        ThreadPool pool;

        auto task1 = std::bind(&calculate, 10, 5);
        std::future<int> future1 = pool.submit(task1);

        auto task2 = std::bind(&do_some_thing);
        std::future<void> future2 = pool.submit(task2);

        std::cout << "waiting for result" << std::endl;

        int result1 = future1.get();

        std::cout << "get result1: " << result1 << std::endl;
        future2.get();
        BOOST_REQUIRE_EQUAL (15, result1); // basic test
    }

BOOST_AUTO_TEST_SUITE_END()