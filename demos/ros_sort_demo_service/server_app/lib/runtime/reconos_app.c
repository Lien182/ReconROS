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



#include "reconos_app.h"

#include "reconos.h"
#include "utils.h"



/* == Application resources ============================================ */

/*
 * @see header
 */
struct mbox resources_address_s;
struct mbox *resources_address = &resources_address_s;

struct mbox resources_acknowledge_s;
struct mbox *resources_acknowledge = &resources_acknowledge_s;









struct ros_node_t resources_node_1_s;
struct ros_node_t *resources_node_1 = &resources_node_1_s;



struct ros_subscriber_t resources_subdata_s;
struct ros_subscriber_t *resources_subdata = &resources_subdata_s;



struct ros_publisher_t resources_pubdata_s;
struct ros_publisher_t *resources_pubdata = &resources_pubdata_s;



std_msgs__msg__UInt32MultiArray resources_sort_msg_s;
std_msgs__msg__UInt32MultiArray *resources_sort_msg = &resources_sort_msg_s;




struct reconos_resource resources_address_res = {
	.ptr = &resources_address_s,
	.type = RECONOS_RESOURCE_TYPE_MBOX
};

struct reconos_resource resources_acknowledge_res = {
	.ptr = &resources_acknowledge_s,
	.type = RECONOS_RESOURCE_TYPE_MBOX
};

struct reconos_resource resources_sort_msg_res = {
	.ptr = &resources_sort_msg_s,
	.type = RECONOS_RESOURCE_TYPE_ROSMSG
};

struct reconos_resource resources_node_1_res = {
	.ptr = &resources_node_1_s,
	.type = RECONOS_RESOURCE_TYPE_ROSNODE
};

struct reconos_resource resources_subdata_res = {
	.ptr = &resources_subdata_s,
	.type = RECONOS_RESOURCE_TYPE_ROSSUB
};

struct reconos_resource resources_pubdata_res = {
	.ptr = &resources_pubdata_s,
	.type = RECONOS_RESOURCE_TYPE_ROSPUB
};




/* == Application functions ============================================ */

/*
 * @see header
 */
void reconos_app_init() {
		mbox_init(resources_address, 128);
		mbox_init(resources_acknowledge, 128);
	

	

	

	

		ros_node_init(resources_node_1, "mynodename");
	

		ros_subscriber_init(resources_subdata, resources_node_1,rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt32MultiArray(), "unsorted", 10000);
	

		ros_publisher_init(resources_pubdata, resources_node_1,rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt32MultiArray(), "sorted");
	

		resources_sort_msg = std_msgs__msg__UInt32MultiArray__create( );
	// VERY; VERY UGLY
	memcpy(&resources_sort_msg_s, resources_sort_msg, sizeof(resources_sort_msg_s));
	resources_sort_msg = &resources_sort_msg_s;
	


	
}

/*
 * @see header
 */
void reconos_app_cleanup() {
		mbox_destroy(resources_address);
		mbox_destroy(resources_acknowledge);
	

	

	

	

		ros_node_destroy(resources_node_1);
	

		ros_subscriber_destroy(resources_subdata);
	

		ros_publisher_destroy(resources_pubdata);
	


		std_msgs__msg__UInt32MultiArray__destroy(resources_sort_msg);
	

	


}

/*
 * Empty software thread if no software specified
 *
 *   data - pointer to ReconOS thread
 */
void *swt_idle(void *data) {
	pthread_exit(0);
}

struct reconos_resource *resources_sortdemo[] = {&resources_address_res,&resources_acknowledge_res,&resources_sort_msg_res,&resources_node_1_res,&resources_subdata_res,&resources_pubdata_res};

/*
 * @see header
 */
struct reconos_thread *reconos_thread_create_hwt_sortdemo(void * init_data) {
	struct reconos_thread *rt = (struct reconos_thread *)malloc(sizeof(struct reconos_thread));
	if (!rt) {
		panic("[reconos-core] ERROR: failed to allocate memory for thread\n");
	}

	int slots[] = {0,1};
	reconos_thread_init(rt, "sortdemo", 0);
	reconos_thread_setinitdata(rt, init_data);
	reconos_thread_setallowedslots(rt, slots, 2);
	reconos_thread_setresourcepointers(rt, resources_sortdemo, 6);
	reconos_thread_create_auto(rt, RECONOS_THREAD_HW);

	return rt;
}


extern void *rt_sortdemo(void *data);

/*
 * @see header
 */
struct reconos_thread *reconos_thread_create_swt_sortdemo(void * init_data, int priority) {
	struct reconos_thread *rt = (struct reconos_thread *)malloc(sizeof(struct reconos_thread));
	if (!rt) {
		panic("[reconos-core] ERROR: failed to allocate memory for thread\n");
	}

	int slots[] = {0,1};
	reconos_thread_init(rt, "sortdemo", 0);
	reconos_thread_setpriority(rt, priority);
	reconos_thread_setinitdata(rt, init_data);
	reconos_thread_setallowedslots(rt, slots, 2);
	reconos_thread_setresourcepointers(rt, resources_sortdemo, 6);
	reconos_thread_setswentry(rt, rt_sortdemo);
	reconos_thread_create_auto(rt, RECONOS_THREAD_SW);

	return rt;
}


/*
 * @see header
 */
void reconos_thread_destroy_sortdemo(struct reconos_thread *rt) {
	// not implemented yet
}



/*
 * @see header
 */
int reconos_clock_system_set(int f)
{
	return reconos_clock_set(0, 8, f);
}

/*
 * @see header
 */
int reconos_clock_threads_set(int f)
{
	return reconos_clock_set(1, 12, f);
}


