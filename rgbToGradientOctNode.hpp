/**

  Represents an octree node which is keyed on an rgb value
  and which stores an row-,col- gradient for mapping colors
  to normals.
  
  Compatible with the libkdtree++ library.

  MIT CSAIL Robot Locomotion Group - geronm 2016

**/

#ifndef __RGB_TO_GRADIENT_OCT_NODE__
#define __RGB_TO_GRADIENT_OCT_NODE__

#define my_abs(a) (((a) > 0) ? (a) : -(a))
#define my_max(a,b) (((a) > (b)) ? (a) : (b))

struct rgbToGradientOctNode
{
  typedef double value_type;

  double xyz[3];
  size_t index;
  double rcgradient[2];

  value_type operator[](size_t n) const
  {
    // this operator is what allows kdtree++ to
    // use the values stored in this node.
    return xyz[n];
  }
 
  double distance( const rgbToGradientOctNode &node)
  {
    double x = xyz[0] - node.xyz[0];
    double y = xyz[1] - node.xyz[1];
    double z = xyz[2] - node.xyz[2];

    // this is what kdtree checks with find_within_range()
    // the "manhattan distance" from the search point.
    // effectively, distance is the maximum distance in any one dimension.
    return my_max(my_abs(x),my_max(my_abs(y),my_abs(z)));
  }
};

#endif // __RGB_TO_GRADIENT_OCT_NODE__
