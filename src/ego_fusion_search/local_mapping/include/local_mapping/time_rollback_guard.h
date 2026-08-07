#ifndef LOCAL_MAPPING_TIME_ROLLBACK_GUARD_H_
#define LOCAL_MAPPING_TIME_ROLLBACK_GUARD_H_

namespace local_mapping {

class TimeRollbackGuard {
 public:
  TimeRollbackGuard();

  bool observe(double timestamp);

 private:
  bool has_timestamp_;
  double last_timestamp_;
};

}  // namespace local_mapping

#endif  // LOCAL_MAPPING_TIME_ROLLBACK_GUARD_H_
