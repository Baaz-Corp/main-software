float[] capture_data_rotation(bool new_data_to_store, float distance, int angle) { //-- add a new data flag in lidar handling function
  if (new_data_to_store) {
    float array1[360] = {}
    array1[angle] = distance;
  }
  return array1;
}

int find_optimal_obstacle_avoidance_angle(float[] arr) {

  int optimal_angle = 0;  //-- optimal angle to turn is initially 0 degrees
  int largest_distance = 0;
  
  for (int i = 0; i < 360; i++) {
      float dist = arr[i+1] - arr[i];
      if (dist > largest_distance) {
        
        largest_distance = dist;
        optimal_angle = i;  //-- if distance difference is large between an angle
        //-- then we could attempt to transverse this area. Store the angle (0, to 360) and use
        //-- the new distance to 

        //-- how this algarithm works idk if it even will, but worth a shot lol
        
      }
  }

  return optimal_angle; //-- returns the angle that the roomba should turn too to avoid the obsticle
  
  
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
