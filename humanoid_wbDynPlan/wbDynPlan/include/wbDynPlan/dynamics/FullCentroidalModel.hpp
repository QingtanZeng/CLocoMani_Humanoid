



/**
* compute the floating-base velocities by joints' angle, velocities, and centroidal momentum 
*/
void computerBaseVelocitiesFromCentroidalModel(const vector_t& state, const vector_t& input,
                                    Vector6& baseVelocity);
