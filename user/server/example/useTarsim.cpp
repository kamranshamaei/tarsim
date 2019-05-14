#include "tarsim.h"
int main(int argc, char **argv) {
  std::string path_to_folder_that_has_win_and_rbs = "/path/to/config";
  tarsim::Tarsim sim(path_to_folder_that_has_win_and_rbs);
  sim.start();
	return 0;
}
