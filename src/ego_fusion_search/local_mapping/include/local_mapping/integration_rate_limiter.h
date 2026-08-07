#ifndef LOCAL_MAPPING_INTEGRATION_RATE_LIMITER_H_
#define LOCAL_MAPPING_INTEGRATION_RATE_LIMITER_H_

namespace local_mapping {

class IntegrationRateLimiter {
 public:
  explicit IntegrationRateLimiter(double rate_hz);

  bool due(double timestamp) const;
  void markIntegrated(double timestamp);

 private:
  double period_;
  bool has_integration_;
  double last_integration_timestamp_;
};

}  // namespace local_mapping

#endif  // LOCAL_MAPPING_INTEGRATION_RATE_LIMITER_H_
