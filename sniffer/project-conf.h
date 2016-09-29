#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER framer_sniffer_802154

#ifdef UIP_CONF_ROUTER
#undef UIP_CONF_ROUTER
#endif
#define UIP_CONF_ROUTER			0

#define RPL_CONF_LEAF_ONLY              1

/* The RF channel must be between 11 and 26 */
//#undef RF_CHANNEL
//#define RF_CHANNEL                              0
