/*
 * Copyright (C) 2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define SDE_HW_KCAL_ENABLED		(1)

#define SDE_HW_KCAL_MIN_VALUE		(20)
#define SDE_HW_KCAL_INIT_PCC		(256)
#define SDE_HW_KCAL_INIT_HUE		(0)
#define SDE_HW_KCAL_INIT_ADJ		(255)

struct sde_hw_kcal_pcc {
	u32 red;
	u32 green;
	u32 blue;
};

struct sde_hw_kcal_hsic {
	u32 hue;
	u32 saturation;
	u32 value;
	u32 contrast;
};

struct sde_hw_kcal_ops {
	void			(*adjust_pcc)	(u32 *data, int plane);
	struct drm_msm_pa_hsic	(*get_hsic)	(void);
};

struct sde_hw_kcal {
	struct sde_hw_kcal_pcc *pcc;
	struct sde_hw_kcal_hsic *hsic;
	struct sde_hw_kcal_ops *ops;

	bool enabled;

	u32 min_cap;
};

struct sde_hw_kcal *sde_hw_kcal_get(void);