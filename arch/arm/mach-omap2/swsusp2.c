#ifdef CONFIG_HIBERNATION

#define OMAP_MUX_BASE_SZ		(0x5cc/4)


u32 padconf[OMAP_MUX_BASE_SZ];
u32 padconf2[0X5C/4];

static void padconf_save(void)
{
	int i;
	for (i=0; i<= 0x234/4; i++)
		padconf[i] = omap_ctrl_readl(OMAP2_CONTROL_PADCONFS+i*4);
	for (i=0x570/4; i<OMAP_MUX_BASE_SZ; i++)
		padconf[i] = omap_ctrl_readl(OMAP2_CONTROL_PADCONFS+i*4);

	for (i=0; i<0X5C/4; i++)
		padconf2[i] = omap_ctrl_readl(OMAP343X_CONTROL_PADCONFS_WKUP+i*4);

}

static void padconf_restore(void)
{
	int i;
	for (i=0; i<= 0x234/4; i++)
		omap_ctrl_writel(padconf[i], OMAP2_CONTROL_PADCONFS+i*4);
	for (i=0x570/4; i<OMAP_MUX_BASE_SZ; i++)
		omap_ctrl_writel(padconf[i], OMAP2_CONTROL_PADCONFS+i*4);

	for (i=0; i<0X5C/4; i++)
		omap_ctrl_writel(padconf2[i], OMAP343X_CONTROL_PADCONFS_WKUP+i*4);

}

u32 pcrm_ctx[5376/4];
static void pcrm_save(void)
{
	int i;
	void *addr = OMAP2_L4_IO_ADDRESS(0x48306000);
	for (i=0; i < 5376/4; i++)
		pcrm_ctx[i] = readl(addr+i*4);

}



void omap_sram_idle_hib(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	int dss_next_state = PWRDM_POWER_ON;
	int dss_state_modified = 0;
	int mpu_next_state = PWRDM_POWER_ON;
	int per_next_state = PWRDM_POWER_ON;
	int core_next_state = PWRDM_POWER_ON;
	int core_prev_state, per_prev_state;
	u32 sdrc_pwr = 0;
	int per_state_modified = 0;
	int per_context_saved = 0;

	if (!_omap_sram_idle)
		return;

	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);
	pwrdm_clear_all_prev_pwrst(dss_pwrdm);

	mpu_next_state = PWRDM_POWER_OFF;
	switch (mpu_next_state) {
	case PWRDM_POWER_ON:
	case PWRDM_POWER_RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PWRDM_POWER_OFF:
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}
	pwrdm_pre_transition();

	/* NEON control */
	if (pwrdm_read_pwrst(neon_pwrdm) == PWRDM_POWER_ON)
		pwrdm_set_next_pwrst(neon_pwrdm, mpu_next_state);

	/* Enable IO-PAD and IO-CHAIN wakeups */
	per_next_state = PWRDM_POWER_OFF;
	core_next_state = PWRDM_POWER_OFF;
	dss_next_state = PWRDM_POWER_OFF;

	if (per_next_state < PWRDM_POWER_ON ||
			core_next_state < PWRDM_POWER_ON) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD, PM_WKEN);
		omap3_enable_io_chain();
	}

	/* DSS */
	if(dss_next_state < PWRDM_POWER_ON){
		if(dss_next_state == PWRDM_POWER_OFF){
			if(core_next_state == PWRDM_POWER_ON){
				dss_next_state = PWRDM_POWER_RET;
				pwrdm_set_next_pwrst(dss_pwrdm, dss_next_state);
				dss_state_modified = 1;
			}
			/* allow dss sleep */
			clkdm_add_sleepdep(dss_pwrdm->pwrdm_clkdms[0],
					mpu_pwrdm->pwrdm_clkdms[0]);
		}else{
			dss_next_state = PWRDM_POWER_RET;
			pwrdm_set_next_pwrst(dss_pwrdm, dss_next_state);
		}
	}

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		if (per_next_state == PWRDM_POWER_OFF) {
			if (core_next_state == PWRDM_POWER_ON) {
				per_next_state = PWRDM_POWER_RET;
				pwrdm_set_next_pwrst(per_pwrdm, per_next_state);
				per_state_modified = 1;
			}
		}
		if (per_next_state == PWRDM_POWER_OFF)
			per_context_saved = 1;

		omap2_gpio_prepare_for_idle(per_context_saved);
		omap_uart_prepare_idle(2);
	}

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		//omap_uart_prepare_idle(0);
		omap_uart_prepare_idle(1);
		if (core_next_state == PWRDM_POWER_OFF) {
			prm_set_mod_reg_bits(OMAP3430_AUTO_OFF_MASK,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET);
			omap3_core_save_context();
			padconf_save();
			omap3_cm_save_context();

			if (omap_rev() < OMAP3630_REV_ES1_2)
				/* Save MUSB context */
				musb_context_save_restore(save_context);
			if (omap_type() != OMAP2_DEVICE_TYPE_GP)
				omap3_save_secure_ram_context(mpu_next_state);
		} else {
			prm_set_mod_reg_bits(OMAP3430_AUTO_RET_MASK,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET);

			if (omap_rev() < OMAP3630_REV_ES1_2)
				musb_context_save_restore(disable_clk);
		}
	}

	omap3_intc_prepare_idle();

	/*
	* On EMU/HS devices ROM code restores a SRDC value
	* from scratchpad which has automatic self refresh on timeout
	* of AUTO_CNT = 1 enabled. This takes care of errata 1.142.
	* Hence store/restore the SDRC_POWER register here.
	*/
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_pwr = sdrc_read_reg(SDRC_POWER);

	/*
	 * omap3_arm_context is the location where ARM registers
	 * get saved. The restore path then reads from this
	 * location and restores them back.
	 */
	//_omap_sram_idle(omap3_arm_context, save_state);
	extern void __save_processor_state(void * ctx, int state);
	__save_processor_state(omap3_arm_context, 4);
	cpu_init();

	/* Restore normal SDRC POWER settings */
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_write_reg(sdrc_pwr, SDRC_POWER);

	/* Restore table entry modified during MMU restoration */
	if (1 || pwrdm_read_prev_pwrst(mpu_pwrdm) == PWRDM_POWER_OFF)
		restore_table_entry();

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		core_prev_state = pwrdm_read_prev_pwrst(core_pwrdm);
		if (1 || core_prev_state == PWRDM_POWER_OFF) {
			omap3_core_restore_context();
			padconf_restore();
			omap3_cm_restore_context();
			omap3_sram_restore_context();
			omap2_sms_restore_context();

			if (omap_rev() < OMAP3630_REV_ES1_2)
				/* Restore MUSB context */
				musb_context_save_restore(restore_context);
		} else {
			if (omap_rev() < OMAP3630_REV_ES1_2)
				musb_context_save_restore(enable_clk);
		}
		//omap_uart_resume_idle(0);
		omap_uart_resume_idle(1);
		if (core_next_state == PWRDM_POWER_OFF)
			prm_clear_mod_reg_bits(OMAP3430_AUTO_OFF_MASK,
					       OMAP3430_GR_MOD,
					       OMAP3_PRM_VOLTCTRL_OFFSET);
		else
			prm_clear_mod_reg_bits(OMAP3430_AUTO_RET_MASK,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET);
	}
	cm_write_mod_reg((1 << OMAP3430_AUTO_PERIPH_DPLL_SHIFT) |
				(1 << OMAP3430_AUTO_CORE_DPLL_SHIFT),
				PLL_MOD,
				CM_AUTOIDLE);

	omap3_intc_resume_idle();

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		omap_uart_resume_idle(2);
		per_prev_state = pwrdm_read_prev_pwrst(per_pwrdm);
		omap2_gpio_resume_after_idle(per_context_saved);

		if (per_state_modified)
			pwrdm_set_next_pwrst(per_pwrdm, PWRDM_POWER_OFF);
	}

	/* DSS */
	if (dss_next_state < PWRDM_POWER_ON) {
		if (dss_next_state == PWRDM_POWER_OFF){
			/* return to the previous state. */
			clkdm_del_sleepdep(dss_pwrdm->pwrdm_clkdms[0],
					mpu_pwrdm->pwrdm_clkdms[0]);
		}
		if (dss_state_modified)
			pwrdm_set_next_pwrst(dss_pwrdm, PWRDM_POWER_OFF);
	}

	/* Disable IO-PAD and IO-CHAIN wakeup */
	if (core_next_state < PWRDM_POWER_ON) {
		prm_clear_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD, PM_WKEN);
		omap3_disable_io_chain();
	}

	pwrdm_post_transition();
}

static int omap3_pm_suspend_hib(void)
{
	struct power_state *pwrst;
	int state, ret = 0;

	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node)
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
	/* Set ones wanted by suspend */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (set_pwrdm_state(pwrst->pwrdm, pwrst->next_state))
			goto restore;
		if (pwrdm_clear_all_prev_pwrst(pwrst->pwrdm))
			goto restore;
	}

	omap_uart_prepare_suspend();
	omap3_intc_suspend();

	omap_sram_idle_hib();

restore:
	/* Restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		if (state > pwrst->next_state) {
			printk(KERN_INFO "Powerdomain (%s) didn't enter "
			       "target state %d\n",
			       pwrst->pwrdm->name, pwrst->next_state);
			ret = -1;
		}
		set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
	}
	if (ret)
		printk(KERN_ERR "Could not enter target state in pm_suspend\n");
	else
		printk(KERN_INFO "Successfully put all powerdomains "
		       "to target state\n");

	return ret;
}

int notrace swsusp_arch_suspend(void)
{
	extern u32 resume_arm_context;
	resume_arm_context = virt_to_phys(omap3_arm_context);
	omap3_pm_suspend_hib();

	/* restore scratchpad */
	omap3_save_scratchpad_contents();

	omap_push_sram_idle();
	return 0;
}

int __naked swsusp_arch_resume(void)
{
	/* we never go here... */
	return 0;
}

#endif
