#include <cstdio>

#include "nura/config/runtime_profile.h"
#include "nura/contracts/trace_topics.h"
#include "nura/flight/flight_policy.h"

namespace c = nura::contracts;
namespace config = nura::config;

namespace
{
const char *stateName(c::FlightState state)
{
    switch (state)
    {
    case c::FlightState::INIT: return "INIT";
    case c::FlightState::SAFE: return "SAFE";
    case c::FlightState::ARMED: return "ARMED";
    case c::FlightState::LAUNCH: return "LAUNCH";
    case c::FlightState::COAST: return "COAST";
    case c::FlightState::APOGEE: return "APOGEE";
    case c::FlightState::DROGUE: return "DROGUE";
    case c::FlightState::DEPLOY: return "DEPLOY";
    case c::FlightState::GROUND: return "GROUND";
    case c::FlightState::FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

const char *domainName(config::ExecutionDomain domain)
{
    switch (domain)
    {
    case config::ExecutionDomain::RECOVERY: return "recovery";
    case config::ExecutionDomain::MISSION: return "mission";
    case config::ExecutionDomain::CRITICAL_SENSOR: return "critical_sensor";
    case config::ExecutionDomain::OPTIONAL_SENSOR: return "optional_sensor";
    case config::ExecutionDomain::SUPERVISOR: return "supervisor";
    case config::ExecutionDomain::EVENT_RECORDER: return "event_recorder";
    case config::ExecutionDomain::TRACE_EXPORTER: return "trace_exporter";
    default: return "unknown";
    }
}
} // namespace

int main(int argc, char **argv)
{
    std::FILE *output = stdout;
    if (argc > 1)
    {
        output = std::fopen(argv[1], "w");
        if (output == nullptr)
        {
            return 2;
        }
    }

    std::fprintf(output,
                 "# Generated Runtime App Catalog\n\n"
                 "Mission profile revision: `%u`<br>\n"
                 "Runtime profile revision: `%u`<br>\n"
                 "Pyro pulse: `%u ms`\n\n",
                 static_cast<unsigned>(
                     nura::flight::FlightPolicy::kProfileRevision),
                 static_cast<unsigned>(config::kRuntimeProfileRevision),
                 static_cast<unsigned>(
                     nura::flight::FlightPolicy::kPyroFireDurationMs));
    std::fprintf(output,
                 "## Scheduled Applications\n\n"
                 "| App | Domain | Period ms | Deadline us | Priority | Stack bytes | Independent thread |\n"
                 "|---|---|---:|---:|---:|---:|---|\n");
    for (const config::AppDescriptor &descriptor : config::kAppCatalog)
    {
        if (descriptor.id == c::AppId::SYSTEM ||
            (!descriptor.independently_scheduled &&
             descriptor.id != c::AppId::INPUT_AGGREGATOR))
        {
            continue;
        }
        std::fprintf(output, "| %s | %s | %u | %u | %d | %u | %s |\n",
                     c::appName(descriptor.id), domainName(descriptor.domain),
                     static_cast<unsigned>(descriptor.period_ms),
                     static_cast<unsigned>(descriptor.deadline_us),
                     static_cast<int>(descriptor.priority),
                     static_cast<unsigned>(descriptor.stack_bytes),
                     descriptor.independently_scheduled ? "yes" : "no");
    }

    std::fprintf(output,
                 "\n## FSM State Applications\n\n"
                 "| State | Enabled state application |\n"
                 "|---|---|\n");
    for (uint8_t raw_state = 0U; raw_state < 10U; ++raw_state)
    {
        const auto state = static_cast<c::FlightState>(raw_state);
        const char *state_app = "none";
        for (const config::AppDescriptor &descriptor : config::kAppCatalog)
        {
            if (!descriptor.independently_scheduled &&
                descriptor.id != c::AppId::INPUT_AGGREGATOR &&
                descriptor.id != c::AppId::SYSTEM &&
                config::isApplicationEnabled(descriptor.id, state))
            {
                state_app = c::appName(descriptor.id);
                break;
            }
        }
        std::fprintf(output, "| %s | %s |\n", stateName(state), state_app);
    }

    if (output != stdout)
    {
        std::fclose(output);
    }
    return 0;
}
