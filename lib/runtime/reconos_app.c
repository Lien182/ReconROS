/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Application library
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Auto-generated application specific header file
 *                 including definitions of all resources and functions
 *                 to instantiate resources and threads automatically.
 *
 * ======================================================================
 */

<<reconos_preproc>>

#include "reconos_app.h"

#include "reconos.h"
#include "utils.h"



/* == Application resources ============================================ */

/*
 * @see header
 */
<<generate for RESOURCES(Type == "mbox")>>
struct mbox <<NameLower>>_s;
struct mbox *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "sem")>>
sem_t <<NameLower>>_s;
sem_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "mutex")>>
pthread_mutex_t <<NameLower>>_s;
pthread_mutex_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "cond")>>
pthread_cond_t <<NameLower>>_s;
pthread_cond_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "rosnode")>>
struct ros_node_t <<NameLower>>_s;
struct ros_node_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "rossub")>>
struct ros_subscriber_t <<NameLower>>_s;
struct ros_subscriber_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "rospub")>>
struct ros_publisher_t <<NameLower>>_s;
struct ros_publisher_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "rossrvs")>>
struct ros_service_server_t <<NameLower>>_s;
struct ros_service_server_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "rosactions")>>
struct ros_action_server_t <<NameLower>>_s;
struct ros_action_server_t *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES(Type == "rosmsg" or Type == "rossrvmsgreq" or Type == "rossrvmsgres" or Type == "rosactionmsggoalreq" or Type == "rosactionmsgresultres" or Type == "rosactionmsgfeedback")>>
<<ROSDataType>> <<NameLower>>_s;
<<ROSDataType>> *<<NameLower>> = &<<NameLower>>_s;
<<end generate>>

<<generate for RESOURCES>>
struct reconos_resource <<NameLower>>_res = {
	.ptr = &<<NameLower>>_s,
	.type = RECONOS_RESOURCE_TYPE_<<TypeUpper>>
};
<<end generate>>

/* == Application functions ============================================ */

/*
 * @see header
 */
void reconos_app_init() {
	<<generate for RESOURCES(Type == "mbox")>>
	mbox_init(<<NameLower>>, <<Args>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "sem")>>
	sem_init(<<NameLower>>, <<Args>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "mutex")>>
	pthread_mutex_init(<<NameLower>>, NULL);
	<<end generate>>

	<<generate for RESOURCES(Type == "cond")>>
	pthread_cond_init(<<NameLower>>, NULL);
	<<end generate>>

	<<generate for RESOURCES(Type == "rosnode")>>
	ros_node_init(<<NameLower>>, <<Args>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rossub")>>
	ros_subscriber_init(<<NameLower>>, <<Group>>_<<Args>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rospub")>>
	ros_publisher_init(<<NameLower>>, <<Group>>_<<Args>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rosmsg" or Type == "rossrvmsgreq" or Type == "rossrvmsgres" or Type == "rosactionmsggoalreq" or Type == "rosactionmsgresultres" or Type == "rosactionmsgfeedback")>>
	<<NameLower>> = <<ROSDataTypeInitFunc>>(<<ROSDataTypeSequenceLength>>);
	// VERY; VERY UGLY
	memcpy(&<<NameLower>>_s, <<NameLower>>, sizeof(<<NameLower>>_s));
	<<NameLower>> = &<<NameLower>>_s;
	<<end generate>>

	<<generate for RESOURCES(Type == "rossrvs")>>
	ros_service_server_init(<<NameLower>>, <<Group>>_<<Args>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rosactions")>>
	ros_action_server_init(<<NameLower>>, <<Group>>_<<Args>>);
	<<end generate>>
	
}

/*
 * @see header
 */
void reconos_app_cleanup() {
	<<generate for RESOURCES(Type == "mbox")>>
	mbox_destroy(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "sem")>>
	sem_destroy(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "mutex")>>
	pthread_mutex_destroy(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "cond")>>
	pthread_cond_destroy(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rosnode")>>
	ros_node_destroy(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rossub")>>
	ros_subscriber_destroy(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rospub")>>
	ros_publisher_destroy(<<NameLower>>);
	<<end generate>>


	<<generate for RESOURCES(Type == "rosmsg" or Type == "rossrvmsgreq" or Type == "rossrvmsgres" or Type == "rosactionmsggoalreq" or Type == "rosactionmsgresultres" or Type == "rosactionmsgfeedback")>>
	<<ROSDataTypeDeInitFunc>>(<<NameLower>>);
	<<end generate>>

	<<generate for RESOURCES(Type == "rossrvs")>>
	ros_service_server_destroy(<<NameLower>>);
	<<end generate>>
	
	<<generate for RESOURCES(Type == "rosactions")>>
	ros_action_server_destroy(<<NameLower>>);
	<<end generate>>

}

/*
 * Empty software thread if no software specified
 *
 *   data - pointer to ReconOS thread
 */
void *swt_idle(void *data) {
	pthread_exit(0);
}

<<generate for THREADS>>
struct reconos_resource *resources_<<Name>>[] = {<<Resources>>};

<<=generate for HasHw=>>
/*
 * @see header
 */
struct reconos_thread *reconos_thread_create_hwt_<<Name>>(void * init_data) {
	struct reconos_thread *rt = (struct reconos_thread *)malloc(sizeof(struct reconos_thread));
	if (!rt) {
		panic("[reconos-core] ERROR: failed to allocate memory for thread\n");
	}

	int slots[] = {<<Slots>>};
	reconos_thread_init(rt, "<<Name>>", 0);
	reconos_thread_setinitdata(rt, init_data);
	reconos_thread_setallowedslots(rt, slots, <<SlotCount>>);
	reconos_thread_setresourcepointers(rt, resources_<<Name>>, <<ResourceCount>>);
	reconos_thread_create_auto(rt, RECONOS_THREAD_HW);

	return rt;
}
<<=end generate=>>

<<=generate for HasSw=>>
extern void *rt_<<Name>>(void *data);

/*
 * @see header
 */
struct reconos_thread *reconos_thread_create_swt_<<Name>>(void * init_data, int priority) {
	struct reconos_thread *rt = (struct reconos_thread *)malloc(sizeof(struct reconos_thread));
	if (!rt) {
		panic("[reconos-core] ERROR: failed to allocate memory for thread\n");
	}

	int slots[] = {<<Slots>>};
	reconos_thread_init(rt, "<<Name>>", 0);
	reconos_thread_setpriority(rt, priority);
	reconos_thread_setinitdata(rt, init_data);
	reconos_thread_setallowedslots(rt, slots, <<SlotCount>>);
	reconos_thread_setresourcepointers(rt, resources_<<Name>>, <<ResourceCount>>);
	reconos_thread_setswentry(rt, rt_<<Name>>);
	reconos_thread_create_auto(rt, RECONOS_THREAD_SW);

	return rt;
}
<<=end generate=>>

/*
 * @see header
 */
void reconos_thread_destroy_<<Name>>(struct reconos_thread *rt) {
	// not implemented yet
}
<<end generate>>

<<generate for CLOCKS>>
/*
 * @see header
 */
int reconos_clock_<<NameLower>>_set(int f)
{
	return reconos_clock_set(<<Id>>, <<M>>, f);
}
<<end generate>>
