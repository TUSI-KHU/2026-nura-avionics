#pragma once

struct SystemHealthState
{
    bool highAccelOk = false;
    bool magOk = false;
    bool storageOk = false;
    bool pyroContinuityOk = false;
    bool deployFired = false;
};
