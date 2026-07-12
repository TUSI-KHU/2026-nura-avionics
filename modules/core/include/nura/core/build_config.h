#pragma once

// Host defaults. Zephyr board/application Kconfig exports these as compiler
// definitions so memory budgets remain a platform profile decision.
#ifndef NURA_TRACE_CAPACITY_RECORDS
#define NURA_TRACE_CAPACITY_RECORDS 2048U
#endif

#ifndef NURA_COMMAND_QUEUE_SLOTS
#define NURA_COMMAND_QUEUE_SLOTS 9U
#endif

#ifndef NURA_ACTUATION_QUEUE_SLOTS
#define NURA_ACTUATION_QUEUE_SLOTS 17U
#endif

#ifndef NURA_TRANSITION_QUEUE_SLOTS
#define NURA_TRANSITION_QUEUE_SLOTS 17U
#endif

#ifndef NURA_DECISION_QUEUE_SLOTS
#define NURA_DECISION_QUEUE_SLOTS 65U
#endif

#ifndef NURA_SENSOR_FAULT_QUEUE_SLOTS
#define NURA_SENSOR_FAULT_QUEUE_SLOTS 9U
#endif

#ifndef NURA_TRACE_EXPORT_BATCH_RECORDS
#define NURA_TRACE_EXPORT_BATCH_RECORDS 128U
#endif
