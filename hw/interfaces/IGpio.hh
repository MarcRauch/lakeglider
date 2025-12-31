#ifndef GL_HW_INTERFACES_IGPIO_H_
#define GL_HW_INTERFACES_IGPIO_H_

namespace gl::hw {

/**
Interface for GPIO functionality. Assumes one instance per pin.
*/
class IGpio {
 public:
  /**
   * Set the pin to high
   */
  virtual void setHigh() = 0;

  /**
   * Set the pin to low
   */
  virtual void setLow() = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_IGPIO_H_
