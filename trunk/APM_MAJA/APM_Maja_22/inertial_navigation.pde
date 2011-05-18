
#if INERTIAL_NAVIGATION == ENABLED

void inertialNavigation()
{    

}

void reached_waypoint()
{
  char message[30];
  sprintf(message,"Reached Waypoint #%i",g.waypoint_index);
  gcs.send_text(SEVERITY_LOW,message);
  g.waypoint_index++;
  wp_index++;
  //increment_WP_index();  
  prev_WP = current_loc;
  next_WP = get_wp_with_index(g.waypoint_index);
  reset_I();

}

void load_first_WP()
{   gcs.send_text(SEVERITY_LOW,"Load First nav waypoint");
    g.waypoint_index =1;
    wp_index = 1;
    prev_WP = current_loc;
    next_WP = get_wp_with_index(g.waypoint_index);
    reset_I();

}

void check_nav_parameters()
{
  
}
#endif
