#include <boost/test/unit_test.hpp>
#include <adap_samples_input/Dummy.hpp>

using namespace adap_samples_input;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    adap_samples_input::DummyClass dummy;
    dummy.welcome();
}
