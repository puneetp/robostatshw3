

void getsensormodel(particle_type particle , map_type map ,float laser_readings[], float weight_value )
{
   //Check if you are in greay space , if yes return 0 weight 

   //For the values X and Y and Tetha project Laser out into the MAP.

   //Read the MAP in these laser rays and get back values of first obstruction as per the map,
   //...rest weight out the gray values if they come before.

   //Use the value to a) Map Reading of lasters b) Laser reading to fetch value c) The value fetched call this in loop 180 times

   //Return the sum of log of all the probabilties ( CHECK again here for what calculation has to be done.)
	 
	int map_directed_obstacle_range[180];
	float per_particle_sensor_probability_vector[180];

	int search_increment=5;
	int hop=5;// How many lasers do we want to hop in search space 

	

	for ( int i ; i<180;i+hop)
	{
		int x=0;
		int y=0;
		int r=0;
		while (x<8000 & y <8000)
		{

			offset_phi = i*pi/180.0; // here degree
			x=std::floor(r*std::cos(theta+offset_phi))
			y=std::floor(r*std::sin(theta+offset_phi))
			
			obstacle_prob=map.prob[x][y];
			if (obstacle_prob==1)
			{
				map_directed_obstacle_range[i]=r;
				break; // check this command 
			}
			r=r+search_increment;
		}	
	}

	// call the function
    ProbabilityDistributionFunction( map_directed_obstacle_range[i], hop, laser_readings[], per_particle_sensor_probability_vector[] )


	for (int j =0 ; j < len(per_particle_sensor_probability_vector) ; j++)
	{
		weight_log=std::log(per_particle_sensor_probability_vector[j])+weight_log;
	}

	weight_value= std::exp(weight_log);  //actual weight 

}


// Matlab equivalent is one line plot /(x,max(exp(-x),normpdf(x,9,.5))) // produce medium peaky graphs
void ProbabilityDistributionFunction( int map_directed_obstacle_range[],int hop,int laser_readings[],int per_particle_sensor_probability_vector )
{
	float mean;
	float var=0.5; //editable
	float min_probability_setting=0.02; //editable
	float exp_value=0.0;
	float normal_value=0.0;
	
	for( int i ; i < map_directed_obstacle_range[] ; i++ )
	{
			//create normal distribution
			mean=map_directed_obstacle_range[i];
				
			// fetch reading at value
			fetch_laser_value_at_this_point = laser_readings[i];

			//create normal distribution
			std::normal_distribution<double> distribution (mean, var);

			// From Normal Distrubution of PDF
			normal_value = distribution(fetch_laser_value_at_this_point);	 

			// From Exponential Function of PDF
			exp_value=std::exp(-1.0*fetch_laser_value_at_this_point);		

			// add some minor noise
			float max_step1=std::max(exp_value,min_probability_setting);
			// get the maximum of the two
			per_particle_sensor_probability_vector[i]=std::max(max_step1,normal_value);

	}

}

