/*
  MCP3221.h - analog input from a MCP3221 I2C ADC

  Driver code for STM32F4xx processors

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

bool MCP3221_init (void);
uint16_t MCP3221_read (void);

/*EOF*/
