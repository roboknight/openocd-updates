/***************************************************************************
 *   Copyright (C) 2013 by Brandon Warhurst																 *
 *   roboknight@gmail+openocd.com                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef JEM_T_H
#define JEM_T_H

#include <jtag/jtag.h>

#define MAX_HARD_BREAKS 2

struct mcu_jtag {
	struct jtag_tap *tap;
	uint8_t dr[48],
			  ir[4];
};

struct jem_common {
	struct mcu_jtag jtag_info;
	int target_started;
	int num_hw_bkpts_avail;
};

#endif /* JEM_T_H */
