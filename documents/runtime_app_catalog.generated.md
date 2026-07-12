# Generated Runtime App Catalog

Mission profile revision: `1`<br>
Runtime profile revision: `1`<br>
Pyro pulse: `1000 ms`

## Scheduled Applications

| App | Domain | Period ms | Deadline us | Priority | Stack bytes | Independent thread |
|---|---|---:|---:|---:|---:|---|
| low_g_sensor | critical_sensor | 10 | 1000 | 2 | 2048 | yes |
| high_g_sensor | critical_sensor | 10 | 1000 | 2 | 2048 | yes |
| barometer_sensor | critical_sensor | 50 | 1500 | 2 | 2048 | yes |
| magnetometer_sensor | optional_sensor | 100 | 1000 | 3 | 2048 | yes |
| gnss_sensor | optional_sensor | 50 | 1000 | 3 | 2048 | yes |
| power_sensor | optional_sensor | 100 | 1000 | 3 | 2048 | yes |
| safety_input | critical_sensor | 10 | 500 | 2 | 1536 | yes |
| input_aggregator | mission | 10 | 1500 | 1 | 0 | no |
| flight_coordinator | mission | 10 | 3000 | 1 | 4096 | yes |
| recovery_actuation | recovery | 2 | 1000 | 0 | 3072 | yes |
| event_recorder | event_recorder | 20 | 2000 | 8 | 3072 | yes |
| supervisor | supervisor | 50 | 1000 | 4 | 3072 | yes |
| trace_exporter | trace_exporter | 20 | 0 | 9 | 3072 | yes |

## FSM State Applications

| State | Enabled state application |
|---|---|
| INIT | none |
| SAFE | none |
| ARMED | launch_detector |
| LAUNCH | burnout_detector |
| COAST | apogee_detector |
| APOGEE | drogue_sequence |
| DROGUE | main_deploy_detector |
| DEPLOY | landing_sequence |
| GROUND | none |
| FAULT | none |
