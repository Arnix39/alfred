// Copyright (c) 2023 Arnix Robotix
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOCK__RASPBERRYPI_MOCK_UTILITIES_HPP_
#define MOCK__RASPBERRYPI_MOCK_UTILITIES_HPP_

#include <cstdint>

int32_t getPwmFrequency(unsigned gpio);
int32_t getPwmDutycycle(unsigned gpio);

#endif  // MOCK__RASPBERRYPI_MOCK_UTILITIES_HPP_
