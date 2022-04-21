#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include <message_logger/message_logger.hpp>

#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

class DataAccessFixture : public ::testing::Test {
 public:
  struct Parameters {
    std::array<int, 2> myDynamicParameter_{0, 0};
  };

  ThreadSafeDataWrapper<Parameters> parameters_;

  static inline void sleepInMilliseconds(unsigned long ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

  //! Number of reads to execute per read thread.
  const int numReads_{100};

  //! Number of writes to execute per write thread.
  const int numWrites_{20};

  // Setup two concurrent reading threads.
  void readerThreadFunction(const char* threadName) {
    for (int i{0}; i < numReads_; i++) {
      Parameters params{parameters_.getData()};
      sleepInMilliseconds(30);
      MELO_DEBUG_STREAM(threadName << " read: [" << params.myDynamicParameter_[0] << ", " << params.myDynamicParameter_[1] << "]\n");
    }
  }

  /**
   * @param updateWriteFunction the write update to perform. See malfunctioningWrite, functioningWrite.
   */
  void writerThreadFunction(std::function<void()> updateWriteFunction) {  // NOLINT readability-make-member-function-const
    for (int i{0}; i < numWrites_; i++) {
      updateWriteFunction();
    }
  }

  /**
   * Adds +1 to the parameter at the provided index.
   * This implementation does not prevent data races among multiple writers. For a correct implementation look at functioningWrite.
   * @param threadName Name to identify the thread in printouts.
   * @param idx Which parameter index to update.
   */
  void malfunctioningWrite(const char* threadName, int idx) {
    Parameters updatedParameters{parameters_.getData()};

    updatedParameters.myDynamicParameter_[idx]++;

    // Mimic some long ongoing update parameter operation.
    sleepInMilliseconds(5);
    parameters_.setData(updatedParameters);

    MELO_DEBUG_STREAM(threadName << " wrote: [" << updatedParameters.myDynamicParameter_[0] << ", "
                                 << updatedParameters.myDynamicParameter_[1] << "]\n");
  }

  /**
   * Adds +1 to the parameter at the provided index.
   * To update a value atomically, use getDataToWrite, which prevents concurrent write updates.
   * Note. The following pattern (see malfunctioningWrite):
   * getData();
   * <updating the value>;
   * setData();
   * is not atomic, thus updates from concurrent threads may be missed.
   * @param threadName Name to identify the thread in printouts.
   * @param idx Which parameter index to update.
   * @return the written value.
   */
  void functioningWrite(const char* threadName, int idx) {
    auto [updatedParameters, parameterGuard]{parameters_.getDataToWrite()};

    updatedParameters.myDynamicParameter_[idx]++;

    // Mimic some long ongoing update parameter operation.
    sleepInMilliseconds(5);

    MELO_DEBUG_STREAM(threadName << " wrote: [" << updatedParameters.myDynamicParameter_[0] << ", "
                                 << updatedParameters.myDynamicParameter_[1] << "]\n");
  }

  void testForDataRace(bool isDataRaceExpected) {
    if (isDataRaceExpected) {
      ASSERT_TRUE(parameters_.getData().myDynamicParameter_[0] != numWrites_ or parameters_.getData().myDynamicParameter_[1] != numWrites_);
    } else {
      ASSERT_EQ(parameters_.getData().myDynamicParameter_[0], numWrites_);
      ASSERT_EQ(parameters_.getData().myDynamicParameter_[1], numWrites_);
    }
  }

  std::vector<std::thread> threads_;

  void joinAllThreads() {
    std::for_each(threads_.begin(), threads_.end(), [](auto& thread) { thread.join(); });
  }
};

TEST_F(DataAccessFixture, TestNonConcurrentReadAndWriteNonDataRacePreventingWrite) {  // NOLINT
  MELO_DEBUG("\n\nTesting Non-Concurrent read and writes: \n\n");

  threads_.emplace_back([&]() {
    readerThreadFunction("Reader1");
    readerThreadFunction("Reader2");
    // Although the malfunctioningWrite does not prevent a write data-race, we do not expect any write data race as there is only one
    // writer at a time.
    writerThreadFunction([&] { return malfunctioningWrite("Writer1", 0); });
    writerThreadFunction([&] { return malfunctioningWrite("Writer2", 1); });
  });
  joinAllThreads();
  testForDataRace(false);
}

TEST_F(DataAccessFixture, TestNonConcurrentReadAndWriteDataRacePreventingWrite) {  // NOLINT
  MELO_DEBUG("\n\nTesting Non-Concurrent read and writes: \n\n");

  threads_.emplace_back([&]() {
    readerThreadFunction("Reader1");
    readerThreadFunction("Reader2");
    writerThreadFunction([&] { return functioningWrite("Writer1", 0); });
    writerThreadFunction([&] { return functioningWrite("Writer2", 1); });
  });
  joinAllThreads();
  testForDataRace(false);
}

TEST_F(DataAccessFixture, TestMalunctioningConcurrentReadAndWrite) {  // NOLINT
  MELO_DEBUG("\n\nTesting Concurrent read and writes: \n\n");

  threads_.emplace_back([&]() { readerThreadFunction("Reader1"); });
  threads_.emplace_back([&]() { readerThreadFunction("Reader2"); });
  threads_.emplace_back([&]() { writerThreadFunction([&] { return malfunctioningWrite("Writer1", 0); }); });
  threads_.emplace_back([&]() { writerThreadFunction([&] { return malfunctioningWrite("Writer2", 1); }); });
  joinAllThreads();
  testForDataRace(true);
}

TEST_F(DataAccessFixture, TestFunctioningConcurrentReadAndWrite) {  // NOLINT
  MELO_DEBUG("\n\nTesting Concurrent read and writes: \n\n");

  threads_.emplace_back([&]() { readerThreadFunction("Reader1"); });
  threads_.emplace_back([&]() { readerThreadFunction("Reader2"); });
  threads_.emplace_back([&]() { writerThreadFunction([&] { return functioningWrite("Writer1", 0); }); });
  threads_.emplace_back([&]() { writerThreadFunction([&] { return functioningWrite("Writer2", 1); }); });
  joinAllThreads();
  testForDataRace(false);
}

}  // namespace elevation_mapping
