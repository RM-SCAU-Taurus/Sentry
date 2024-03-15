#include "bsp_shoot_class.h"
#include "shoot_task.h"


	shoot_class_parent_t PROTECT_choice={.mode =PROTECT_MODE, 
																.Fric_action = Fric_protect,
																.Trigger_action = Trigger_STOP_or_PROTECT,
																};	


	shoot_class_child_t RC_UP_choice={.base.mode =PROTECT_MODE, 
																			.base.Fric_action = Fric_protect,
																			.base.Trigger_action = Trigger_STOP_or_PROTECT,
																			};
		
	shoot_class_child_t RC_MI_choice= {.base.mode =UNPROTECT_MODE, 
																			.base.Fric_action = Fric_unprotect,
																			.base.Trigger_action = Trigger_STOP_or_PROTECT,
																			};
		
	shoot_class_child_t RC_DN_choice={.base.mode =UNPROTECT_MODE, 
																			.base.Fric_action = Fric_unprotect,
																			.base.Trigger_action = Trigger_SINGLE_or_SERIES,
																			};		
		

		
		
		
		