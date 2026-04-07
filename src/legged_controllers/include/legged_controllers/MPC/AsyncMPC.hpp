#ifndef ASYNCMPC_HPP
#define ASYNCMPC_HPP

#include <thread>
#include <mutex>
#include <atomic>
#include "legged_controllers/MPC/MPC.hpp"
#include "legged_trajectory/GaitScheduler.hpp"
#include "legged_common/DataTypes.hpp"

class AsyncMPC {
public:
    // Kurucu: MPC objesini ve gerekli parametreleri alır
    AsyncMPC(double dt, int iterBetweenMPC);
    ~AsyncMPC();

    // Ana döngüden (Go2MPC) her iterasyonda çağrılacak fonksiyon.
    // Sadece verileri kopyalar, hesaplamayı beklemez.
    void updateMPCState(const DesiredStates& desiredStates, const EstimatorData& estData, Gait* gait);

    // Ana döngünün hesaplanmış güçleri alması için kullanılır.
    std::array<Eigen::Vector3d, 4> getLatestFootForces();

    // Threadi başlatma ve durdurma
    void start();
    void stop();

private:
    void threadLoop(); // Arka planda dönecek olan asıl fonksiyon

    ModelPredictiveControl* mpc_;
    
    std::thread worker_thread_;
    std::mutex data_mutex_;
    std::atomic<bool> stop_thread_{false};

    // --- Paylaşımlı Veriler (Ana döngü -> MPC) ---
    DesiredStates shared_desired_states_;
    EstimatorData shared_est_data_;
    Gait* shared_gait_{nullptr};

    // --- Paylaşımlı Veriler (MPC -> Ana döngü) ---
    std::array<Eigen::Vector3d, 4> shared_foot_forces_;
};

#endif // MPCTHREAD_HPP