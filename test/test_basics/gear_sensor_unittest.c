#include <gear_sensor.h>
#include <stdio.h>
#include <unity.h>

void test_forward()
{
    gear_sensor_state_t state = {};
    state.threshold = 10;
    state.decay_exp = 8;

    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1000, 2000), 0);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1000, 2100), 1);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1000, 2100), 0);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1100, 2100), 1);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1100, 2100), 0);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1100, 2000), 1);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1100, 2000), 0);
    TEST_ASSERT_EQUAL(gear_sensor_process(&state, 1100, 2000), 0);
}

void test_forward_backward()
{
    gear_sensor_state_t state = {};
    state.threshold = 10;
    state.decay_exp = 8;

    TEST_ASSERT_EQUAL( 0, gear_sensor_process(&state, 1000, 2000));
    TEST_ASSERT_EQUAL( 1, gear_sensor_process(&state, 1000, 2100));
    TEST_ASSERT_EQUAL( 0, gear_sensor_process(&state, 1000, 2100));
    TEST_ASSERT_EQUAL( 1, gear_sensor_process(&state, 1100, 2100));
    TEST_ASSERT_EQUAL( 0, gear_sensor_process(&state, 1100, 2100));
    TEST_ASSERT_EQUAL(-1, gear_sensor_process(&state, 1000, 2100));
    TEST_ASSERT_EQUAL(-1, gear_sensor_process(&state, 1000, 2000));
}

void test_decay()
{
    gear_sensor_state_t state = {};
    state.threshold = 10;
    state.decay_exp = 4;

    TEST_ASSERT_EQUAL(0, gear_sensor_process(&state, 5000, 10000));

    TEST_ASSERT_EQUAL(10000, state.min_B);
    TEST_ASSERT_EQUAL(10000, state.max_B);

    for (int i = 0; i < 1000; i++)
    {
        TEST_ASSERT_EQUAL(0, gear_sensor_process(&state, 5000, 9900));
    }

    TEST_ASSERT_EQUAL(9900, state.max_B);
    TEST_ASSERT_EQUAL(9900, state.min_B);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_forward);
    RUN_TEST(test_forward_backward);
    RUN_TEST(test_decay);

    UNITY_END();

    return 0;
}

