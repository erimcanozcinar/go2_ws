#include "legged_controllers/MPC/AsyncMPC.hpp"

AsyncMPC::AsyncMPC(double dt, int iterBetweenMPC) {
    mpc_ = new ModelPredictiveControl(&shared_est_data_, dt, iterBetweenMPC);
    for(int i=0; i<4; i++) shared_foot_forces_[i].setZero();
}

AsyncMPC::~AsyncMPC() {
    stop();
    delete mpc_;
}

void AsyncMPC::start() {
    stop_thread_ = false;
    worker_thread_ = std::thread(&AsyncMPC::threadLoop, this);
}

void AsyncMPC::stop() {
    stop_thread_ = true;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

// Ana döngü (Go2MPC::update) bu fonksiyonu çağırıp sadece veriyi bırakacak
void AsyncMPC::updateMPCState(const DesiredStates& desiredStates, const EstimatorData& estData, Gait* gait) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    shared_desired_states_ = desiredStates;
    shared_est_data_ = estData;
    shared_gait_ = gait;
}

// Ana döngü (Go2MPC::update) bu fonksiyonla en taze sonucu alacak
std::array<Eigen::Vector3d, 4> AsyncMPC::getLatestFootForces() {
    std::array<Eigen::Vector3d, 4> current_forces;
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_forces = shared_foot_forces_;
    return current_forces;
}

// Arka planda sürekli dönen hesaplama döngüsü
void AsyncMPC::threadLoop() {
    while (!stop_thread_) {
        DesiredStates local_des_states;
        Gait* local_gait = nullptr;
        
        {
            // Verilerin kopyasını al
            std::lock_guard<std::mutex> lock(data_mutex_);
            local_des_states = shared_desired_states_;
            local_gait = shared_gait_;
            // shared_est_data_ zaten MPC oluşturulurken pointer olarak verildiği için
            // mpc objesi updateData() vb. fonksiyonlarında doğrudan en güncel veriyi okuyacaktır.
        }

        if (local_gait != nullptr) {
            mpc_->setDesiredStates(&local_des_states);
            mpc_->run(local_gait);

            // Sonuçları güvenle kaydet
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                shared_foot_forces_ = mpc_->getFootForces();
            }
        }
        
        // İşlemciyi yormamak için kısa bir bekleme
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}