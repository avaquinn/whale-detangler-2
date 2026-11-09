// out_of_bounds.cpp
int controlled_out_of_bounds_read() {
  int values[2] = {0, 0};
  return values[1];
}
