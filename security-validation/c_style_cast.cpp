// c_style_cast.cpp
int main() {
  int value = 42;
  void* raw = &value;
  return *static_cast<int*>(raw);
}
