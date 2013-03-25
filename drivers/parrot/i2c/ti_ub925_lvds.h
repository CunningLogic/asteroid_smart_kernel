#ifdef CONFIG_PARROT_MCLAREN_INPUT
int
lvds_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags1,
	    const char *name, void *dev);
void lvds_free_irq(unsigned int irq, void *dev_id);
#else
static inline lvds_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags1, const char *name, void *dev)
{
	return -EINVAL;
}

static inline void lvds_free_irq(unsigned int irq, void *dev_id)
{
}
#endif
