// security-validation/sast/cppcheck/null_pointer.cpp
int controlled_null_pointer() {
  int* value = nullptr;
  return *value;
}
