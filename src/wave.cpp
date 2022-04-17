#include <iostream>

#include <cassert>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <numbers>
#include <cmath>
#include <complex>

#include "ocelot/djinni/cpp/Wave.hpp"
#include "ocelot/djinni/cpp/CoreType.hpp"

#include <Eigen/Dense>

namespace ocelot {

class WaveSingle : public djinni::Wave {
public:
	~WaveSingle() override = default;

	void prop() override {
	}
};
}  // namespace ocelot

namespace ocelot::djinni {
std::shared_ptr<Wave> Wave::createWave(CoreType type) {
	switch (type) {
	case CoreType::SINGLE:
		return std::make_shared<ocelot::WaveSingle>();
	default:
		return nullptr;
	}
}

void meshgrid(Eigen::Map<Eigen::VectorXd> &vecX, Eigen::Map<Eigen::VectorXd> &vecY, Eigen::MatrixXd &meshX, Eigen::MatrixXd &meshY)
{
    const auto vecXLength = vecX.size();
    const auto vecYLength = vecY.size();
    meshX.resize(vecXLength, vecYLength);
    meshY.resize(vecYLength, vecXLength);
    std::cerr << "meshX.size()\n" << meshX.size() << "\n" << "meshY.size()\n" << meshY.size() << std::endl;
    for (int i = 0; i < vecYLength; ++i)
    {	
	meshX.row(i) = vecX;
	//std::cout << "meshX.row("<< i <<")\n" << meshX.row(i)<< std::endl;
    }

    for (int i = 0; i < vecXLength; ++i)
    {
	meshY.col(i) = vecY.transpose();
	//std::cout << "meshX.row(" << i << ")\n" << meshY.col(i) << std::endl;
    }
}

using MatrixType = Eigen::MatrixXcd;
using KVectorType = Eigen::Map<Eigen::VectorXd>;
using KMatrixType = Eigen::MatrixXd;

class UndulatorField {
public:
    virtual ~UndulatorField() = default;
    virtual MatrixType calculate(const KMatrixType& x, const KMatrixType& y, float l_xi, float l_yi, float eta_xi, float eta_yi, float z, float l_w, int32_t E_ph, float C, std::optional<float> phi) = 0;
};

class UndulatorFieldFar : public UndulatorField {
public:
    MatrixType calculate(const KMatrixType& x, const KMatrixType& y, float l_xi, float l_yi, float eta_xi, float eta_yi, float z, float l_w, int32_t E_ph, float C, std::optional<float> phi) override;
};

class UndulatorFieldNear : public UndulatorField {
public:
    MatrixType calculate(const KMatrixType& x, const KMatrixType& y, float l_xi, float l_yi, float eta_xi, float eta_yi, float z, float l_w, int32_t E_ph, float C, std::optional<float> phi) override;
};

constexpr auto speed_of_light = 299792458.0; // m/s
constexpr auto q_e = 1.6021766208e-19;       // C - Elementary charge
//m_e_kg = 9.10938215e-31      # kg
constexpr auto h_J_s = 6.626070040e-34;      // Plancks constant [J*s]

//m_e_eV = m_e_kg * speed_of_light**2 / q_e  # eV (510998.8671)
//m_e_MeV = m_e_eV / 1e+6                    # MeV (0.510998928)
//m_e_GeV = m_e_eV / 1e+9                    # GeV

//mu_0 = 4 * pi * 1e-7                     # permeability of free space (1.2566370614359173e-06)
//epsilon_0 = 1 / mu_0 / speed_of_light**2 # permittivity of free space (8.854187817620e-12 F/m)

constexpr auto h_eV_s = h_J_s / q_e;                     // [eV*s]
constexpr auto hr_eV_s = h_eV_s/2./std::numbers::pi;

MatrixType UndulatorFieldFar::calculate(const KMatrixType& theta_x, const KMatrixType& theta_y, float l_x, float l_y, float eta_x, float eta_y, float z, float L_w, int32_t E_ph, float C, std::optional<float> phi) {
    const auto lambda_w = 0.04;
    const auto w = E_ph / hr_eV_s;
    const double z_0 = z;

    std::cerr << "c: theta_x.rows() = " << theta_x.rows() << "theta_x.cols() = " << theta_x.cols() << std::endl;
    std::cerr << "c: theta_y.rows() = " << theta_y.rows() << "theta_y.cols() = " << theta_y.cols() << std::endl;
    MatrixType E = -(
        L_w * C / 2 +
        L_w * w * (
            Eigen::square(theta_x.array() -
             (l_x/z_0) -
             eta_x) +
            Eigen::square(theta_y.array() - (l_y/z_0) - eta_y)) / 4 /speed_of_light / std::numbers::pi).unaryExpr([](double d) {
                return d == 0 ? 1 : std::cos(d)/d;
            }).matrix()
                             * Eigen::exp(
                            std::complex<double>(0.0, 1.0) * w  * z_0 * (Eigen::square(theta_x.array() - l_x/z_0) + Eigen::square(theta_y.array() - (l_y/z_0))).cast<std::complex<double>>() / 2 / speed_of_light).matrix();
    std::cerr << "c: E.rows() = " << E.rows() << "E.cols() = " << E.cols() << std::endl;

    if (phi) {
//        _logger.debug(ind_str + 'adding a random phase to the field')
        E *= std::exp(std::complex<double>(0.0, 1.0) * std::complex<double>(*phi));
    }
/*
    lambda_w = 0.04
    w = E_ph / hr_eV_s

    _logger.debug(ind_str + 'calculating undulator field in the far field approximation')

    if z != 0: 
        z_0 = z
    else: 
        raise AttributeError('"phi" is bool type ')
        _logger.error('"z" must not be zero')
    
    start_time = time.time()
    E = -np.sinc(
        L_w * C / 2 +
        L_w * w * (
            (theta_x -
             (l_x/z_0) -
             eta_x)**2 +
            (theta_y - (l_y/z_0) - eta_y)**2) / 4 /speed_of_light / np.pi)
                             * np.exp(
                            1i * w  * z * ((theta_x - (l_x/z))**2 + (theta_y - (l_y/z_0))**2) / 2 / speed_of_light)
    if phi != None:
        _logger.debug(ind_str + 'adding a random phase to the field')
        E = E * np.exp(1j * phi)
    
    _logger.debug(ind_str + 'done in {:.2f} seconds'.format(time.time() - start_time))
    return E        
*/
    return E.matrix();
}

MatrixType UndulatorFieldNear::calculate(const KMatrixType& x, const KMatrixType& y, float l_xi, float l_yi, float eta_xi, float eta_yi, float z, float l_w, int32_t E_ph, float C, std::optional<float> phi) {
    return MatrixType();
}

class PhiProvider {
public:
    virtual ~PhiProvider() = default;
    virtual std::optional<float> phi() = 0;
};

class NonePhiProvider : public PhiProvider {
public:
    std::optional<float> phi() override {
        return std::nullopt;
    }
};

class UniformRandomPhiProvider : public PhiProvider {
public:
    UniformRandomPhiProvider(std::mt19937& gen)
        : dis(0.0, std::numbers::pi_v<float>)
        , m_gen(gen)
    {}
    std::optional<float> phi() override {
        return dis(m_gen);
    }
private:
    std::uniform_real_distribution<float> dis;
    std::mt19937& m_gen;
};

std::unique_ptr<UndulatorField> getField(const std::string& approximation) {
    if (approximation == "far_field") {
        return std::make_unique<UndulatorFieldFar>();
    } else if (approximation == "near_field") {
        return std::make_unique<UndulatorFieldNear>();
    } else {
        throw std::logic_error("attribute 'method' must be 'far_field' or 'near_field'");
    }
}

std::unique_ptr<PhiProvider> getPhiProvider(const std::string& mode, std::mt19937& gen) {
    if (mode == "incoh") {
        return std::make_unique<UniformRandomPhiProvider>(gen);
    } else if (mode == "coh") {
        return std::make_unique<NonePhiProvider>();
    } else {
        throw std::logic_error("attribute 'mode' must be 'ihcos' or 'coh'");
    }
}

std::vector<uint8_t> Wave::mul(const std::vector<uint8_t> & _x, const std::vector<uint8_t> & _y) {
    const size_t xSize = std::sqrt(_x.size()/16);
    Eigen::Map<Eigen::MatrixXcd> x((std::complex<double> *)_x.data(), xSize, xSize);

    const size_t ySize = std::sqrt(_y.size()/16);
    Eigen::Map<Eigen::MatrixXcd> y((std::complex<double> *)_y.data(), ySize, ySize);

    const Eigen::MatrixXcd r = x.matrix() * y.matrix();

    std::vector<uint8_t> result(xSize * xSize * 16);
    memcpy(result.data(), r.data(), xSize * xSize * 16);
    return result;
}

std::vector<uint8_t> Wave::undulator_field_dfl_MP_int(const std::vector<uint8_t> & _kx, const std::vector<uint8_t> & _ky, int32_t Ny, int32_t Nx, float z, float L_w, int32_t E_ph, int32_t N_b, int32_t N_e, float sig_x, float sig_y, float sig_xp, float sig_yp, float C, const std::string & approximation, const std::string & mode) {
    std::cerr << "Wave::undulator_field_dfl_MP_int: z = " << z << std::endl;

    KVectorType kx((double *)_kx.data(), _kx.size()/8);
    KVectorType ky((double *)_ky.data(), _ky.size()/8);

    KMatrixType x, y;

    meshgrid(kx, ky, x, y);

    std::cerr << "kx.rows() = " << kx.rows() << "kx.cols() = " << kx.cols() << std::endl;

    std::vector<MatrixType> result2(N_b);

    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::unique_ptr<UndulatorField> field = getField(approximation);
    std::unique_ptr<PhiProvider> phiProvider = getPhiProvider(mode, gen);
    
    std::normal_distribution<float> l_x_dis{0.0, sig_x};
    std::normal_distribution<float> l_y_dis{0.0, sig_y};
    std::normal_distribution<float> eta_x_dis{0.0, sig_xp};
    std::normal_distribution<float> eta_y_dis{0.0, sig_yp};

    int32_t i = 1;
    for (auto& E : result2) {
        E = MatrixType::Zero(Ny, Nx);

        for (int32_t j = 1; j <= N_e; ++j) {
            E += field->calculate(x, y, l_x_dis(gen), l_y_dis(gen), eta_x_dis(gen), eta_y_dis(gen), z, L_w, E_ph, C, phiProvider->phi());
            if (N_b == 1) {
                std::cerr << "electrons simulated = " << j << " out of " << N_e << std::endl;
            }
        }
        std::cerr << "realizations number = " << i << " out of " << N_b << std::endl;
        ++i;
    }

    std::vector<uint8_t> result;
    result.reserve(16);
    ((float *)(result.data()))[0] = 1.f;
    ((float *)(result.data()))[1] = 2.f;
    ((float *)(result.data()))[2] = 3.f;
    ((float *)(result.data()))[3] = 4.f;

    return result;
}

}  // namespace ocelot::djinni
