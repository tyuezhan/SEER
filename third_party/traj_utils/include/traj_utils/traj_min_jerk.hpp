/*
    MIT License

    Copyright (c) 2020 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef TRAJ_MIN_JERK_HPP
#define TRAJ_MIN_JERK_HPP

#include "root_finder.hpp"

#include <Eigen/Eigen>

#include <iostream>
#include <cmath>
#include <vector>

namespace min_jerk
{

    // Polynomial order and trajectory dimension are fixed here
    constexpr int TrajOrder = 5;
    constexpr int TrajDim = 3;

    // Type for piece boundary condition and coefficient matrix
    typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> BoundaryCond;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> CoefficientMat;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder> VelCoefficientMat;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder - 1> AccCoefficientMat;

    // A single piece of a trajectory, which is indeed a polynomial
    class Piece
    {
    private:
        // A piece is totally determined by boundary condition and duration
        // boundCond = [p(0),v(0),a(0),p(T),v(T),a(T)]
        BoundaryCond boundCond;
        double duration;
        bool synced;
        // The normalized coefficient is generated from boundCond and duration
        // These members CANNOT be accessed unless through normalizedCoeffMat()
        // p(t) = c5*t^5 + c4*t^4 + ... + c1*t + c0
        // nCoeffMat = [c5*T^5,c4*T^4,c3*T^3,c2*T^2,c1*T,c0*1]
        CoefficientMat nCoeffMat;
        inline void normalizedCoeffMat(void)
        {
            if (!synced)
            {

                double t1 = duration;
                double t2 = t1 * t1;

                // It maps boundary condition to normalized coefficient matrix
                nCoeffMat.col(0) = 0.5 * (boundCond.col(5) - boundCond.col(2)) * t2 -
                                   3.0 * (boundCond.col(1) + boundCond.col(4)) * t1 +
                                   6.0 * (boundCond.col(3) - boundCond.col(0));
                nCoeffMat.col(1) = (-boundCond.col(5) + 1.5 * boundCond.col(2)) * t2 +
                                   (8.0 * boundCond.col(1) + 7.0 * boundCond.col(4)) * t1 +
                                   15.0 * (-boundCond.col(3) + boundCond.col(0));
                nCoeffMat.col(2) = (0.5 * boundCond.col(5) - 1.5 * boundCond.col(2)) * t2 -
                                   (6.0 * boundCond.col(1) + 4.0 * boundCond.col(4)) * t1 +
                                   10.0 * (boundCond.col(3) - boundCond.col(0));
                nCoeffMat.col(3) = 0.5 * boundCond.col(2) * t2;
                nCoeffMat.col(4) = boundCond.col(1) * t1;
                nCoeffMat.col(5) = boundCond.col(0);
            }

            return;
        }

    public:
        Piece() = default;

        // Constructor from boundary condition and duration
        Piece(BoundaryCond bdCond, double dur) : boundCond(bdCond), duration(dur), synced(false)
        {
            normalizedCoeffMat();
        }

        inline int getDim() const
        {
            return TrajDim;
        }

        inline int getOrder() const
        {
            return TrajOrder;
        }

        inline double getDuration() const
        {
            return duration;
        }

        // Get the position at time t in this piece
        inline Eigen::Vector3d getPos(double t)
        {
            // Normalize the time
            t /= duration;
            Eigen::Vector3d pos(0.0, 0.0, 0.0);
            double tn = 1.0;
            for (int i = TrajOrder; i >= 0; i--)
            {
                pos += tn * nCoeffMat.col(i);
                tn *= t;
            }
            // The pos is not affected by normalization
            return pos;
        }

        // Get the velocity at time t in this piece
        inline Eigen::Vector3d getVel(double t)
        {
            // Normalize the time
            t /= duration;
            Eigen::Vector3d vel(0.0, 0.0, 0.0);
            double tn = 1.0;
            int n = 1;
            for (int i = TrajOrder - 1; i >= 0; i--)
            {
                vel += n * tn * nCoeffMat.col(i);
                tn *= t;
                n++;
            }
            // Recover the actual vel
            vel /= duration;
            return vel;
        }

        // Get the acceleration at time t in this piece
        inline Eigen::Vector3d getAcc(double t)
        {
            // Normalize the time
            t /= duration;
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = TrajOrder - 2; i >= 0; i--)
            {
                acc += m * n * tn * nCoeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            // Recover the actual acc
            acc /= duration * duration;
            return acc;
        }

        inline Eigen::Vector3d getJer(double t)
        {
            t /= duration;
            Eigen::Vector3d jer(0.0, 0.0, 0.0);
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = TrajOrder - 3; i >= 0; i--)
            {
                jer += l * m * n * tn * nCoeffMat.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            jer /= duration * duration * duration;
            return jer;
        }

        // Get the boundary condition of this piece
        inline const BoundaryCond &getBoundCond() const
        {
            return boundCond;
        }

        inline const CoefficientMat &getNormalizedCoeffMat() const
        {
            return nCoeffMat;
        }

        // Get the coefficient matrix of the piece
        // Default arg chooses the natural coefficients
        // If normalized version is needed, set the arg true
        inline CoefficientMat getCoeffMat(bool normalized = false)
        {
            CoefficientMat posCoeffsMat;
            double t = 1;
            for (int i = TrajOrder; i >= 0; i--)
            {
                posCoeffsMat.col(i) = nCoeffMat.col(i) / t;
                t *= normalized ? 1.0 : duration;
            }
            return posCoeffsMat;
        }

        // Get the polynomial coefficients of velocity of this piece
        // Default arg chooses the natural coefficients
        // If normalized version is needed, set the arg true
        inline VelCoefficientMat getVelCoeffMat(bool normalized = false)
        {
            VelCoefficientMat velCoeffMat;
            int n = 1;
            double t = 1.0;
            t *= normalized ? 1.0 : duration;
            for (int i = TrajOrder - 1; i >= 0; i--)
            {
                velCoeffMat.col(i) = n * nCoeffMat.col(i) / t;
                n++;
                t *= normalized ? 1.0 : duration;
            }
            return velCoeffMat;
        }

        // Get the polynomial coefficients of acceleration of this piece
        // Default arg chooses the natural coefficients
        // If normalized version is needed, set the arg true
        inline AccCoefficientMat getAccCoeffMat(bool normalized = false)
        {
            AccCoefficientMat accCoeffMat;
            int n = 2;
            int m = 1;
            double t = 1.0;
            t *= normalized ? 1.0 : duration * duration;
            for (int i = TrajOrder - 2; i >= 0; i--)
            {
                accCoeffMat.col(i) = n * m * nCoeffMat.col(i) / t;
                n++;
                m++;
                t *= normalized ? 1.0 : duration;
            }
            return accCoeffMat;
        }

        // Get the max velocity rate of the piece
        inline double getMaxVelRate()
        {
            // Compute normalized squared vel norm polynomial coefficient matrix
            Eigen::MatrixXd nVelCoeffMat = getVelCoeffMat(true);
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getVel(0.0).norm();
            }
            else
            {
                // Search an open interval whose boundaries are not zeros
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                // Find all stationaries
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                // Check boundary points and stationaries within duration
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxVelRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        // Recover the actual time then get the vel squared norm
                        tempNormSqr = getVel((*it) * duration).squaredNorm();
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                    }
                }
                return sqrt(maxVelRateSqr);
            }
        }

        // Get the max acceleration rate of the piece
        inline double getMaxAccRate()
        {
            // Compute normalized squared acc norm polynomial coefficient matrix
            Eigen::MatrixXd nAccCoeffMat = getAccCoeffMat(true);
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getAcc(0.0).norm();
            }
            else
            {
                // Search an open interval whose boundaries are not zeros
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                // Find all stationaries
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                // Check boundary points and stationaries within duration
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        // Recover the actual time then get the acc squared norm
                        tempNormSqr = getAcc((*it) * duration).squaredNorm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }

        // Check whether velocity rate of the piece is always less than maxVelRate
        inline bool checkMaxVelRate(double maxVelRate)
        {
            double sqrMaxVelRate = maxVelRate * maxVelRate;
            if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
                getVel(duration).squaredNorm() >= sqrMaxVelRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nVelCoeffMat = getVelCoeffMat(true);
                Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(2));
                // Convert the actual squared maxVelRate to a normalized one
                double t2 = duration * duration;
                coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
                // Directly check the root existence in the normalized interval
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        // Check whether accleration rate of the piece is always less than maxAccRate
        inline bool checkMaxAccRate(double maxAccRate)
        {
            double sqrMaxAccRate = maxAccRate * maxAccRate;
            if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
                getAcc(duration).squaredNorm() >= sqrMaxAccRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nAccCoeffMat = getAccCoeffMat(true);
                Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(2));
                // Convert the actual squared maxAccRate to a normalized one
                double t2 = duration * duration;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
                // Directly check the root existence in the normalized interval
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }
    };

    // A whole trajectory which contains multiple pieces
    class Trajectory
    {
    private:
        typedef std::vector<Piece> Pieces;
        Pieces pieces;

    public:
        Trajectory() = default;

        // Constructor from boundary conditions and durations
        Trajectory(const std::vector<BoundaryCond> &bdConds,
                   const std::vector<double> &durs)
        {
            int N = std::min(durs.size(), bdConds.size());
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(bdConds[i], durs[i]);
            }
        }

        inline bool is_empty() const
        {
          if (pieces.size() <= 0)
          {
            return true;
          }
        return false;
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        // Get durations vector of all pieces
        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        // Get total duration of the trajectory
        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        // known the idx of the trajectory
        inline Eigen::Vector2d getStartEndTime(int idx) const
        {
            double totalDuration = 0.0;
            Eigen::Vector2d se_time;

            if (idx > 0)
            {
                for (int i = 0; i < idx; i++)
                {
                    totalDuration += pieces[i].getDuration();
                }
            }
            se_time(0) = totalDuration;
            se_time(1) = totalDuration + pieces[idx].getDuration();

            return se_time;
        }

        // this include the start and end point
        inline Eigen::MatrixXd getPositions() const
        {
            int N = getPieceNum();
            Eigen::MatrixXd positions(3, N + 1);
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getBoundCond().col(0);
            }
            positions.col(N) = pieces[N - 1].getBoundCond().col((TrajOrder + 1) / 2);
            return positions;
        }

        inline Eigen::MatrixXd getConstPoints(const int K) const
        {
            // k is the resolution
            int N = getPieceNum();
            Eigen::MatrixXd pts(3, N * K + 1);
            Eigen::Vector3d pos;
            Eigen::Matrix<double, 6, 1> beta0;
            double s1, s2, s3, s4, s5;
            double step;
            int i_dp = 0;
            for (int i = 0; i < N; ++i)
            {
                step = 1.0 / double(K);
                s1 = 0.0;
                double t = 0;
                // innerLoop = K;
                for (int j = 0; j <= K; ++j)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0 << s5, s4, s3, s2, s1, 1.0;
                    pos = pieces[i].getNormalizedCoeffMat() * beta0;
                    pts.col(i_dp) = pos;

                    s1 += step;
                    if (j != K || (j == K && i == N - 1))
                    {
                        ++i_dp;
                    }
                }
            }
            return pts;
        }

        // Reload the operator[] to access the i-th piece
        inline const Piece &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline Pieces::const_iterator begin() const
        {
            return pieces.begin();
        }

        inline Pieces::const_iterator end() const
        {
            return pieces.end();
        }

        inline Pieces::iterator begin()
        {
            return pieces.begin();
        }

        inline Pieces::iterator end()
        {
            return pieces.end();
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        // Put another piece at the tail of this trajectory
        inline void emplace_back(const Piece &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const BoundaryCond &bdCond, const double &dur)
        {
            pieces.emplace_back(bdCond, dur);
            return;
        }

        // Append another Trajectory at the tail of this trajectory
        inline void append(const Trajectory &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        // Find the piece at which the time t is located
        // The index is returned and the offset in t is removed
        inline int locatePieceIdx(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;
        }

        // Get the position at time t of the trajectory
        inline Eigen::Vector3d getPos(double t)
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getPos(t);
        }

        // Get the velocity at time t of the trajectory
        inline Eigen::Vector3d getVel(double t)
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        // Get the acceleration at time t of the trajectory
        inline Eigen::Vector3d getAcc(double t)
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        // Get the jerk at time t of the trajectory
        inline Eigen::Vector3d getJer(double t)
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        // Get the position at the juncIdx-th waypoint
        inline Eigen::Vector3d getJuncPos(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getBoundCond().col(0);
            }
            else
            {
                return pieces[juncIdx - 1].getBoundCond().col((TrajOrder + 1) / 2);
            }
        }

        // Get the velocity at the juncIdx-th waypoint
        inline Eigen::Vector3d getJuncVel(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getBoundCond().col(1);
            }
            else
            {
                return pieces[juncIdx - 1].getBoundCond().col((TrajOrder + 1) / 2 + 1);
            }
        }

        // Get the acceleration at the juncIdx-th waypoint
        inline Eigen::Vector3d getJuncAcc(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getBoundCond().col(2);
            }
            else
            {
                return pieces[juncIdx - 1].getBoundCond().col((TrajOrder + 1) / 2 + 2);
            }
        }

        // Get the max velocity rate of the trajectory
        inline double getMaxVelRate()
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxVelRate();
                maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
            }
            return maxVelRate;
        }

        // Get the max acceleration rate of the trajectory
        inline double getMaxAccRate()
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxAccRate();
                maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
            }
            return maxAccRate;
        }

        // Check whether the velocity rate of this trajectory exceeds the threshold
        inline bool checkMaxVelRate(double maxVelRate)
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
            }
            return feasible;
        }

        // locate whether the velocity rate of this trajectory exceeds the threshold
        inline void locateMaxVelRate(double maxVelRate, std::vector<int> &idx_list)
        {
            idx_list.clear();
            int N = getPieceNum();
            for (int i = 0; i < N; i++)
            {
                if (!pieces[i].checkMaxVelRate(maxVelRate))
                {
                    idx_list.push_back(i);
                }
            }
            return;
        }

        // Check whether the acceleration rate of this trajectory exceeds the threshold
        inline bool checkMaxAccRate(double maxAccRate)
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
            }
            return feasible;
        }

        inline void locateMaxAccRate(double maxAccRate, std::vector<int> &idx_list)
        {
            idx_list.clear();
            int N = getPieceNum();
            for (int i = 0; i < N; i++)
            {
                if (!pieces[i].checkMaxAccRate(maxAccRate))
                {
                    idx_list.push_back(i);
                }
            }
            return;
        }
    };

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int rows = lowerBw + upperBw + 1;
            int actualSize = N * rows;
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            offset = new double *[rows];
            double *ptrRow = ptrData;
            for (int i = 0; i < rows; i++)
            {
                offset[i] = ptrRow;
                ptrRow += N;
            }
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            if (offset != nullptr)
            {
                delete[] offset;
                offset = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;
        double **offset = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return offset[i - j + upperBw][j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return offset[i - j + upperBw][j];
        }

        // This function conducts banded LU factorization in place
        // Note that the matrix "A" MUST NOT HAVE ZERO PIVOTS !!!
        inline void factorizeLU()
        {
            int iM, jM;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                for (int i = k + 1; i <= iM; i++)
                {
                    operator()(i, k) /= operator()(k, k);
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    for (int i = k + 1; i <= iM; i++)
                    {
                        operator()(i, j) -= operator()(i, k) * operator()(k, j);
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
            return;
        }
    };

    class JerkOpt
    {
    public:
        JerkOpt() = default;
        ~JerkOpt() { A.destroy(); }

    private:
        int N;
        Eigen::Matrix3Xd Ps;
        Eigen::Matrix3Xd VAs;
        Eigen::VectorXd T;
        BandedSystem A;
        Eigen::MatrixX3d b;

        // Temp variables
        Eigen::VectorXd t2;
        Eigen::VectorXd t3;
        Eigen::VectorXd t4;
        Eigen::VectorXd t5;
        Eigen::VectorXd cv00, cv01, cv02;
        Eigen::VectorXd cv10, cv11, cv12;
        Eigen::VectorXd cv20, cv21, cv22;
        Eigen::VectorXd ca00, ca01, ca02;
        Eigen::VectorXd ca10, ca11, ca12;
        Eigen::VectorXd ca20, ca21, ca22;
        Eigen::VectorXd w_all, b_all;

    private:
        // only the jerk objective
        inline double evalPieceObjective(const Eigen::Array3d &iP,
                                         const Eigen::Array3d &iV,
                                         const Eigen::Array3d &iA,
                                         const Eigen::Array3d &fP,
                                         const Eigen::Array3d &fV,
                                         const Eigen::Array3d &fA,
                                         const double &duration) const
        {
            Eigen::VectorXd coeffsJerObjective(5);
            coeffsJerObjective(0) = (9.0 * iA.square() - 6.0 * iA * fA + 9.0 * fA.square()).sum();
            coeffsJerObjective(1) = 24.0 * (3.0 * iA * iV - 2.0 * fA * iV + 2.0 * iA * fV - 3.0 * fA * fV).sum();
            coeffsJerObjective(2) = 24.0 * (8.0 * iV.square() + 14.0 * iV * fV + 8.0 * fV.square() + 5.0 * (iA - fA) * (iP - fP)).sum();
            coeffsJerObjective(3) = 720.0 * ((iV + fV) * (iP - fP)).sum();
            coeffsJerObjective(4) = 720.0 * (iP - fP).square().sum();

            double t2 = duration * duration;
            double t5 = t2 * t2 * duration;

            return RootFinder::polyVal(coeffsJerObjective, duration) / t5;
        }

    public:
        inline void reset(const Eigen::Matrix3d &headState,
                          const Eigen::Matrix3d &tailState,
                          const int &pieceNum)
        {
            N = pieceNum;
            Ps.resize(3, N + 1);
            VAs.resize(3, 2 * N + 2);
            Ps.leftCols<1>() = headState.leftCols<1>();
            Ps.rightCols<1>() = tailState.leftCols<1>();
            VAs.leftCols<2>() = headState.rightCols<2>();
            VAs.rightCols<2>() = tailState.rightCols<2>();
            T.resize(N);
            A.create(2 * N - 2, 3, 3);
            b.resize(2 * N - 2, 3);

            cv00.resize(N - 1);
            cv01.resize(N - 1);
            cv02.resize(N - 1);
            cv10.resize(N - 1);
            cv11.resize(N - 1);
            cv12.resize(N - 1);
            cv20.resize(N - 1);
            cv21.resize(N - 1);
            cv22.resize(N - 1);
            ca00.resize(N - 1);
            ca01.resize(N - 1);
            ca02.resize(N - 1);
            ca10.resize(N - 1);
            ca11.resize(N - 1);
            ca12.resize(N - 1);
            ca20.resize(N - 1);
            ca21.resize(N - 1);
            ca22.resize(N - 1);

            return;
        }

        inline void generate(const Eigen::Matrix3Xd &inPs,
                             const Eigen::VectorXd &ts)
        {
            T = ts;
            const Eigen::VectorXd &t1 = T;
            t2 = t1.cwiseProduct(t1);
            t3 = t2.cwiseProduct(t1);
            t4 = t2.cwiseProduct(t2);
            t5 = t4.cwiseProduct(t1);

            // Computed nonzero entries in A and b for linear system Ax=b to be solved
            for (int i = 0; i < N - 1; i++)
            {
                cv00(i) = 720.0 / t4(i);
                cv01(i) = 720.0 * (1.0 / t4(i + 1) - 1.0 / t4(i));
                cv02(i) = -720.0 / t4(i + 1);
                cv10(i) = 336.0 / t3(i);
                cv11(i) = 384.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
                cv12(i) = 336.0 / t3(i + 1);
                cv20(i) = 48.0 / t2(i);
                cv21(i) = 72.0 * (1.0 / t2(i + 1) - 1.0 / t2(i));
                cv22(i) = -48.0 / t2(i + 1);

                ca00(i) = -120.0 / t3(i);
                ca01(i) = 120.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
                ca02(i) = -120.0 / t3(i + 1);
                ca10(i) = -48.0 / t2(i);
                ca11(i) = 72.0 * (1.0 / t2(i + 1) - 1.0 / t2(i));
                ca12(i) = 48.0 / t2(i + 1);
                ca20(i) = -6.0 / t1(i);
                ca21(i) = 18.0 * (1.0 / t1(i + 1) + 1.0 / t1(i));
                ca22(i) = -6.0 / t1(i + 1);
            }

            Ps.block(0, 1, 3, N - 1) = inPs;

            if (N == 2)
            {
                Eigen::Matrix2d invA;
                Eigen::Matrix<double, 2, 3> bl;
                invA.setZero();
                bl.setZero();

                // A = [cv11(0), cv21(0); ca11(0), ca21(0);]
                invA(0, 0) = ca21(0);
                invA(0, 1) = -cv21(0);
                invA(1, 0) = -ca11(0);
                invA(1, 1) = cv11(0);
                invA /= invA(0, 0) * invA(1, 1) - invA(0, 1) * invA(1, 0);
                bl.row(0) = (-cv00(0) * Ps.col(0) - cv01(0) * Ps.col(1) - cv02(0) * Ps.col(2) - cv10(0) * VAs.col(0) - cv20(0) * VAs.col(1) - cv12(0) * VAs.col(4) - cv22(0) * VAs.col(5)).transpose();
                bl.row(1) = (-ca00(0) * Ps.col(0) - ca01(0) * Ps.col(1) - ca02(0) * Ps.col(2) - ca10(0) * VAs.col(0) - ca20(0) * VAs.col(1) - ca12(0) * VAs.col(4) - ca22(0) * VAs.col(5)).transpose();

                VAs.block(0, 2, 3, 2) = (invA * bl).transpose();
            }
            else
            {
                A.reset();
                b.setZero();

                A(0, 0) = cv11(0);
                A(0, 1) = cv21(0);
                A(0, 2) = cv12(0);
                A(0, 3) = cv22(0);
                A(1, 0) = ca11(0);
                A(1, 1) = ca21(0);
                A(1, 2) = ca12(0);
                A(1, 3) = ca22(0);
                A(2 * N - 4, 2 * N - 6) = cv10(N - 2);
                A(2 * N - 4, 2 * N - 5) = cv20(N - 2);
                A(2 * N - 4, 2 * N - 4) = cv11(N - 2);
                A(2 * N - 4, 2 * N - 3) = cv21(N - 2);
                A(2 * N - 3, 2 * N - 6) = ca10(N - 2);
                A(2 * N - 3, 2 * N - 5) = ca20(N - 2);
                A(2 * N - 3, 2 * N - 4) = ca11(N - 2);
                A(2 * N - 3, 2 * N - 3) = ca21(N - 2);

                b.row(0) = (-cv00(0) * Ps.col(0) - cv01(0) * Ps.col(1) - cv02(0) * Ps.col(2) - cv10(0) * VAs.col(0) - cv20(0) * VAs.col(1)).transpose();
                b.row(1) = (-ca00(0) * Ps.col(0) - ca01(0) * Ps.col(1) - ca02(0) * Ps.col(2) - ca10(0) * VAs.col(0) - ca20(0) * VAs.col(1)).transpose();
                b.row(2 * N - 4) = (-cv00(N - 2) * Ps.col(N - 2) - cv01(N - 2) * Ps.col(N - 1) - cv02(N - 2) * Ps.col(N) - cv12(N - 2) * VAs.col(2 * N) - cv22(N - 2) * VAs.col(2 * N + 1)).transpose();
                b.row(2 * N - 3) = (-ca00(N - 2) * Ps.col(N - 2) - ca01(N - 2) * Ps.col(N - 1) - ca02(N - 2) * Ps.col(N) - ca12(N - 2) * VAs.col(2 * N) - ca22(N - 2) * VAs.col(2 * N + 1)).transpose();

                for (int i = 1; i < N - 2; i++)
                {
                    A(i * 2, i * 2 - 2) = cv10(i);
                    A(i * 2, i * 2 - 1) = cv20(i);
                    A(i * 2, i * 2) = cv11(i);
                    A(i * 2, i * 2 + 1) = cv21(i);
                    A(i * 2, i * 2 + 2) = cv12(i);
                    A(i * 2, i * 2 + 3) = cv22(i);
                    A(i * 2 + 1, i * 2 - 2) = ca10(i);
                    A(i * 2 + 1, i * 2 - 1) = ca20(i);
                    A(i * 2 + 1, i * 2) = ca11(i);
                    A(i * 2 + 1, i * 2 + 1) = ca21(i);
                    A(i * 2 + 1, i * 2 + 2) = ca12(i);
                    A(i * 2 + 1, i * 2 + 3) = ca22(i);

                    b.row(i * 2) = (-cv00(i) * Ps.col(i) - cv01(i) * Ps.col(i + 1) - cv02(i) * Ps.col(i + 2)).transpose();
                    b.row(i * 2 + 1) = (-ca00(i) * Ps.col(i) - ca01(i) * Ps.col(i + 1) - ca02(i) * Ps.col(i + 2)).transpose();
                }

                // Solve Ax=b using banded LU factorization
                A.factorizeLU();
                // The solution is computed in place.
                A.solve(b);

                VAs.block(0, 2, 3, 2 * N - 2) = b.transpose();
            }

            return;
        }

        inline double getObjective() const
        {
            double objective = 0.0;

            for (int i = 0; i < N; i++)
            {
                objective += evalPieceObjective(Ps.col(i), VAs.col(2 * i), VAs.col(2 * i + 1),
                                                Ps.col(i + 1), VAs.col(2 * i + 2), VAs.col(2 * i + 3),
                                                T(i));
            }

            return objective;
        }

        // should call before optimization
        inline void setWeights(Eigen::VectorXd wts,
                               Eigen::VectorXd bds)
        {

            w_all = wts;
            // w_time_,  w_vel_,  w_acc_,  w_sta_obs_
            b_all = bds;
            // vel_max, acc_max
        }

        template <typename EIGENVEC, typename EIGENVECGD>
        inline void VTGrad(
            const Eigen::VectorXd &RT, const EIGENVEC &VT,
            const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT)
        {
            for (int i = 0; i < VT.size(); ++i)
            {
                double gdVT2Rt;
                if (VT(i) > 0)
                {
                    gdVT2Rt = VT(i) + 1.0;
                }
                else
                {
                    double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
                    gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
                }

                gdVT(i) = (gdRT(i) + w_all(0)) * gdVT2Rt;
            }
        }

        template <typename EIGENVEC, typename EIGENMAT>
        inline void addPieceCostGrad(EIGENVEC &gdT, // N
                                     EIGENMAT &gdP, // 3, N-1
                                     double &piece_cost,
                                     std::vector<Eigen::MatrixXd> &SFCs)
        {

            double tempT, tempT6;
            Eigen::Matrix<double, 3, 1> P0, V0, A0, P1, V1, A1, P2, V2, A2;
            Eigen::VectorXd costs(4);
            double T1, T2;

            Eigen::Vector3d u_vel, u_pos, dist_vec;
            Eigen::Vector3d info_vec;
            double q_time, dist_val2, dist_offset, dist_offset2, dist_offset3;

            Eigen::Matrix<double, 3, 1> vel, acc;

            costs.setZero();

            // add time gradients
            gdT += w_all(0) * Eigen::VectorXd::Ones(N);


            //std::cout << "gdT " << gdT << std::endl;

            // add w_time * duration
            costs(0) = T.sum();

            // M = N-1

            // i = 1, M-1
            // i = 0, N-2

            double T_now = T(0);

            for (int i = 0; i < N - 1; ++i)
            {
                T1 = T(i);
                T2 = T(i + 1);

                // q0, q1, ..., qN-2
                P0 = Ps.col(i);
                V0 = VAs.col(2 * i);
                // std::cout << "[debug::vel] V0" << V0 << std::endl;
                A0 = VAs.col(2 * i + 1);

                // q1, q2, ..., qN-1
                P1 = Ps.col(i + 1);
                V1 = VAs.col(2 * i + 2);
                A1 = VAs.col(2 * i + 3);
                // std::cout << "[debug::vel] V1" << V1 << std::endl;
                //  q2, q2, ..., qN
                P2 = Ps.col(i + 2);
                V2 = VAs.col(2 * i + 4);
                A2 = VAs.col(2 * i + 5);
                // std::cout << "[debug::vel] V2" << V2 << std::endl;

                vel = (P2 - P0) / (T1 + T2);
                acc = ((P2 - P1) / T2 - (P1 - P0) / T1) / (0.5 * (T1 + T2));

                double g_vel = vel.squaredNorm() - b_all(0) * b_all(0);
                double g_acc = acc.squaredNorm() - b_all(1) * b_all(1);
                // std::cout << "[debug::vel] vel " << vel << std::endl;
                // std::cout << "[debug::vel] acc " << acc << std::endl;

                // // add vel/acc cost
                if (g_vel > 0)
                {
                    // std::cout << "[debug::vel] vel.squaredNorm() " << vel.squaredNorm()<< std::endl;
                    double g_vel_sqr = g_vel * g_vel;
                    double t_total = T1 + T2;
                    // std::cout << "[debug::acc] g_vel_sqr " << g_vel_sqr << std::endl;
                    costs(1) += g_vel_sqr * g_vel;

                    double vel2t = -6 * g_vel_sqr * vel.squaredNorm() / t_total;
                    // std::cout << "[debug::acc] vel2t " << vel2t << std::endl;
                    // gradients to intervals
                    gdT(i) += w_all(1) * vel2t;
                    gdT(i + 1) += w_all(1) * vel2t;

                    // gradients to points
                    Eigen::Matrix<double, 3, 1> tempv = 6 * g_vel_sqr * vel / t_total;
                    // std::cout << "[debug::acc] tempv " << tempv << std::endl;
                    //  std::cout << "[debug::acc] gdP.col(i) " << (gdP.col(i)).size() << std::endl;
                    // std::cout << "[debug::acc] gdP " << gdP<< std::endl;

                    // q0, q1, ..., qN-3
                    if (i > 0)
                    {
                        gdP.col(i - 1) -= w_all(1) * tempv;
                    }

                    // std::cout << "[debug::acc] N " << N << std::endl;
                    // std::cout << "[debug::acc] i " << i << std::endl;

                    // q2, q3, ..., qN-1
                    if (i < N - 2)
                    {
                        gdP.col(i + 1) += w_all(1) * tempv;
                    }
                }

                if (g_acc > 0)
                {
                    // std::cout << "[debug::acc]acc " <<  acc << std::endl;
                    double g_acc_sqr = g_acc * g_acc;
                    double t_total = T1 + T2;

                    costs(2) += g_acc_sqr * g_acc;
                    double acc2t = -6 * g_acc_sqr * acc.squaredNorm() / t_total;
                    // std::cout << "[debug::acc] acc2t " << acc2t << std::endl;
                    // gradients to intervals
                    gdT(i) += w_all(2) * (acc2t + 6 * g_acc_sqr / (T1 * T1) * acc.transpose() * (P1 - P0));
                    gdT(i + 1) += w_all(2) * (acc2t - 6 * g_acc_sqr / (T2 * T2) * acc.transpose() * (P2 - P1));

                    // gradients to points
                    Eigen::Matrix<double, 3, 1> tempa = 6 * g_acc_sqr * acc / (0.5 * t_total);
                    // std::cout << "[debug::acc] tempa " << tempa << std::endl;
                    //  std::cout << "[debug::acc] gdP.col(i) " << (gdP.col(i)).size() << std::endl;
                    // std::cout << "[debug::acc] gdP " << gdP<< std::endl;

                    // gdP  q1 q2 ... qm-1
                    if (i > 0)
                    {
                        gdP.col(i - 1) += w_all(2) * tempa / T1;
                    }

                    gdP.col(i) -= w_all(2) * tempa * (1.0 / T1 + 1.0 / T2);

                    // std::cout << "[debug::acc] i + 2 " << i + 2 << std::endl;
                    if (i < N - 2)
                    {
                        gdP.col(i + 1) += w_all(2) * tempa / T2;
                    }
                }

                // // //size = N-1 i = 0, 1, ..., N-2
                // std::cout << "i is " << i << std::endl;

                for (int idx = i; idx <= i + 1; idx++)
                {

                    unsigned int corr_k = SFCs[idx].cols();
                    for (unsigned int k = 0; k < corr_k; k++)
                    {
                        Eigen::Matrix<double, 3, 1> outerNormal = SFCs[idx].col(k).tail<3>();
                        double tempsfc = outerNormal.dot(P1 - SFCs[idx].col(k).head<3>());
                        if (tempsfc > 0)
                        {
                            //std::cout << "tempsfc + rad_dis is " << tempsfc + rad_dis << std::endl;
                            costs(3) += -std::log(tempsfc);
                            gdP.col(i) += w_all(3) * 1.0 / (tempsfc) * outerNormal;
                        }
                    }
                }

                T_now += T2;
            }

            //std::cout << "[debug::total] costs " << costs << std::endl;

            // std::cout << "gdT " << gdT << std::endl;

            piece_cost = w_all.transpose() * costs;
        }

        inline Eigen::VectorXd getGradT(void) const
        {
            Eigen::VectorXd grad(N);

            double tempT, tempT6;
            Eigen::Array3d iP, iV, iA, fP, fV, fA;
            Eigen::VectorXd coeffsGradT(5);
            for (int i = 0; i < N; i++)
            {
                // Get the information of the piece
                tempT = T(i);
                tempT6 = tempT * tempT;
                tempT6 = tempT6 * tempT6 * tempT6;

                // Calculate the numerator of dJi(T)/dT without time regularization
                iP = Ps.col(i);
                iV = VAs.col(2 * i);
                iA = VAs.col(2 * i + 1);
                fP = Ps.col(i + 1);
                fV = VAs.col(2 * i + 2);
                fA = VAs.col(2 * i + 3);

                coeffsGradT(0) = (-9.0 * iA.square() + 6.0 * iA * fA - 9.0 * fA.square()).sum();
                coeffsGradT(1) = -48.0 * ((3.0 * iA - 2.0 * fA) * iV + (2.0 * iA - 3.0 * fA) * fV).sum();
                coeffsGradT(2) = -72.0 * ((8.0 * iV.square() + 14.0 * iV * fV + 8.0 * fV.square()).sum() +
                                          (5.0 * (iA - fA) * (iP - fP)).sum());
                coeffsGradT(3) = -2880.0 * ((iV + fV) * (iP - fP)).sum();
                coeffsGradT(4) = -3600.0 * (iP - fP).square().sum();

                // Calculate the gradient
                grad(i) = RootFinder::polyVal(coeffsGradT, tempT) / tempT6;
            }

            return grad;
        }

        inline Eigen::Matrix3Xd getGradInnerP(void) const
        {
            Eigen::Matrix3Xd grad(3, N - 1);

            for (int i = 1; i < N; i++)
            {
                grad.col(i - 1) = 120.0 * (-VAs.col(2 * i - 1) / t3(i - 1) + VAs.col(2 * i + 1) * (1.0 / t3(i - 1) + 1.0 / t3(i)) - VAs.col(2 * i + 3) / t3(i)) +
                                  720.0 * (-VAs.col(2 * i - 2) / t4(i - 1) - VAs.col(2 * i) * (1.0 / t4(i - 1) - 1.0 / t4(i)) + VAs.col(2 * i + 2) / t4(i)) +
                                  1440.0 * (-Ps.col(i - 1) / t5(i - 1) + Ps.col(i) * (1.0 / t5(i - 1) + 1.0 / t5(i)) - Ps.col(i + 1) / t5(i));
            }

            return grad;
        }

        inline void getTraj(Trajectory &traj) const
        {
            traj.clear();
            traj.reserve(N);
            BoundaryCond boundCond;

            // std::cout << "[debug::vel] VAs" << VAs << std::endl;
            for (int i = 0; i < N; i++)
            {
                boundCond.col(0) = Ps.col(i);
                boundCond.col(1) = VAs.col(2 * i);
                boundCond.col(2) = VAs.col(2 * i + 1);
                boundCond.col(3) = Ps.col(i + 1);
                boundCond.col(4) = VAs.col(2 * i + 2);
                boundCond.col(5) = VAs.col(2 * i + 3);

                traj.emplace_back(boundCond, T(i));
            }

            return;
        }

        inline void checkCollision() const
        {
        }
    };

} // namespace min_jerk

#endif
