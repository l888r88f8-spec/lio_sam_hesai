// ARM64 portability shim: upstream's BMI2 uses are guarded by __BMI2__.
#pragma once
#if defined(__BMI2__)
#error "ARM64 shim must never be used with BMI2 enabled"
#endif
