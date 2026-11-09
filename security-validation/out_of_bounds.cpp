int controlled_out_of_bounds_read() {
  int values[1] = {0};
  return values[1];
}
