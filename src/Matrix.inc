#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( T value)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) = value;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< T >& aList)
{
	// Check the arguments
	assert( aList.size() == M * N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column, ++row_iter)
		{
			matrix.at( row).at( column) = *row_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< std::initializer_list< T > >& aList)
{
	// Check the arguments, the static assert assures that there is at least 1 M and 1 N!
	assert( aList.size() == M && (*aList.begin()).size() == N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
	{
		auto column_iter = (*row_iter).begin();
		for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
		{
			matrix.at( row).at( column) = *column_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const Matrix< T, M, N >& aMatrix) :
				matrix( aMatrix.matrix)
{
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex)
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex) const
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
T& Matrix< T, M, N >::at( 	std::size_t aRowIndex,
							std::size_t aColumnIndex)
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const T& Matrix< T, M, N >::at( std::size_t aRowIndex,
								std::size_t aColumnIndex) const
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex)
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex) const
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator=( const Matrix< T, M, N >& rhs)
{
	if (this != &rhs)
	{
		matrix = rhs.matrix;
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
bool Matrix< T, M, N >::operator==( const Matrix< T, M, N >& rhs) const
{
	return matrix == rhs.matrix;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator*=( const T2& scalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) *= scalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator*( const T2& scalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result *= scalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator/=( const T2& aScalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) /= aScalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator/( const T2& aScalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result /= aScalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator+=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] += rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator+( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result += rhs;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator-=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] -= rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator-( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template< typename T, std::size_t M, std::size_t N >
template< std::size_t columns >
Matrix< T, M, columns > Matrix< T, M, N >::operator*( const Matrix< T, N, columns >& rhs) const
{
	Matrix< T, M, columns > result;
	if(N!=rhs.getRows())
	{
		std::cout << "These matrices are not multiplicable" << std::endl;
	}
	else
	{
		for(std::size_t row = 0;row<M;row++)
		{
			for(std::size_t column = 0;column<columns;column++)
			{
				T sum = 0;
				for(std::size_t counter = 0;counter<N;counter++)
				{
					sum = sum + matrix[row][counter] * rhs[counter][column];
				}
				result[row][column] = sum;
			}
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, N, M > Matrix< T, M, N >::transpose() const
{
	Matrix< T, N, M > result;
	for(std::size_t row = 0;row<M;row++)
	{
		for(std::size_t column = 0;column<N;column++)
		{
			result[row][column] = matrix[column][row];
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::identity() const
{
	Matrix< T, N, M > result;
	for(std::size_t row = 0;row<M;row++)
	{
		for(std::size_t column = 0;column<N;column++)
		{
			if(row==column)
			{
				result[row][column] = 1;
			}
			else
			{
				result[row][column] = 0;
			}
		}
	}
	return result;
}
/**
 *
 */

template< typename T, std::size_t M, std::size_t N >
Matrix< T, 1, N > Matrix< T, M, N >::calc(const Matrix< T,N,N >& rhs,unsigned short power) const
{
	Matrix< T, N, N > result = rhs.identity();
	Matrix< T, 1, N > statevector = *this;
	for(unsigned short i = 0;i<power;i++)
	{
		result = rhs * result;
	}
	return statevector*result;
}

template< typename T, std::size_t M, std::size_t N >
Matrix< T, 1, N > Matrix< T, M, N >::steady(const Matrix< T,N,N >& rhs) const
{
	Matrix< T, N, N > tempMatrix = rhs;
	Matrix< T, 1, N > startState = *this;
	Matrix< T, 1, N > currentState;
	Matrix< T, 1, N > prevState;
	int counter = 0;
	while(true)
	{
		tempMatrix = tempMatrix * rhs;
		currentState = startState * tempMatrix;
		std::cout<<prevState<<std::endl;
		std::cout<<currentState<<std::endl;
		std::cout<<"----------------"<<std::endl;
		if(equals(currentState,prevState,std::numeric_limits<T>::epsilon(),pow(10,12))) 
		{
			std::cout<<"Steady state reached after "<<counter<<std::endl;
			break;
			
		}
		prevState = currentState;
		counter++;
	}
	return currentState;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gauss() const
{
	Matrix< T, M, N > result;
	Matrix< T, M, N > temp;
	temp = *this;
	result = *this;
	std::size_t highestRow = 0;
	T highestNumber = 0;
	for(std::size_t counter = 0;counter<M;counter++)
	{
		if(counter+1<M)
		{
			for(std::size_t row = counter;row<M;row++)
			{
				if(highestNumber<std::abs(temp[row][counter]))
				{
					highestNumber = std::abs(temp[row][counter]);
					highestRow = row;
				}
			}
			result[counter] = temp[highestRow];
			result[highestRow] = temp[counter];
			temp = result;
		}	
		
		for(std::size_t targetRow = 0;targetRow<counter;targetRow++)
		{
			T pivot = result[counter][targetRow];
			for(std::size_t column = 0;column<N;column++)
			{
				result[counter][column] = result[counter][column]-pivot*result[targetRow][column];
			}
		}

		T pivot = result[counter][counter];

		for(std::size_t column = 0;column<N;column++)
		{
			result[counter][column] = result[counter][column]/pivot;
		}

	}
	
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gaussJordan() const
{
	Matrix< T, M, N > result;
	result = *this;
	result = result.gauss();
	for(std::size_t counter = M-2;counter<M;counter--)
	{
		for(std::size_t targetRow = counter+1;targetRow<M;targetRow++)
		{
			T pivot = result[counter][targetRow];

			for(std::size_t column = 0;column<N;column++)
			{
				result[counter][column] = result[counter][column]-pivot*result[targetRow][column];
			}
		}
	}
	return result;
}
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, 1 > Matrix< T, M, N >::solve() const
{
	Matrix < T, M, 1 > result;
	Matrix <T,M,N> gjResult;
	gjResult = this->gaussJordan();
	for(std::size_t row = 0;row<M;row++)
	{
		result[row][0] = gjResult[row][N-1];
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::inverse() const
{
	Matrix< T, M, N > result;
	Matrix< T, M, N > identity;
	Matrix< T, M, N*2 > resultIdentity;
	identity = this->identity();
	result = *this;
	for(std::size_t row = 0;row<M;row++)
	{
		for(std::size_t column = 0;column<N*2;column++)
		{
			if(column>=N)
			{
				resultIdentity[row][column] = identity[row][column-N];
			}
			else
			{
				resultIdentity[row][column] = result[row][column];
			}
		}
	}
	resultIdentity = resultIdentity.gaussJordan();
	for(std::size_t row = 0;row<M;row++)
	{
		for(std::size_t column = N;column<N*2;column++)
		{
			result[row][column-N] = resultIdentity[row][column];
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::string Matrix< T, M, N >::to_string() const
{
	std::string result = "Matrix<" + std::to_string( N) + "," + std::to_string( M) + ">\n{\n";
	for (std::size_t i = 0; i < M; ++i)
	{
		for (std::size_t j = 0; j < N; ++j)
		{
			result += std::to_string( matrix[i][j]) + ",";
		}
		result += "\n";
	}
	result += "}";
	return result;
}
/**
 *
 */
template< typename T, const std::size_t N >
bool equals(	const Matrix< T, 1, N >& lhs,
				const Matrix< T, 1, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	for(std::size_t column = 0;column<N;column++)
	{
		T diff = std::abs(lhs[0][column]-rhs[0][column]);
		if(diff>aPrecision*aFactor)
		{
			return false;
		}
		//if(lhs[0][column]-aPrecision*aFactor>rhs[0][column]||lhs[0][column]+aPrecision*aFactor<rhs[0][column])
		//{
		//	return false;
		//}
	}
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M >
bool equals(	const Matrix< T, M, 1 >& lhs,
				const Matrix< T, M, 1 >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	for(std::size_t row = 0;row<M;row++)
	{
		T diff = std::abs(lhs[row][0]-rhs[row][0]);
		if(diff>aPrecision*aFactor)
		{
			return false;
		}
		//if(lhs[row][0]-aPrecision*aFactor>rhs[row][0]||lhs[row][0]+aPrecision*aFactor<rhs[row][0])
		//{
		//	return false;
		//}
	}
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M, const std::size_t N >
bool equals(	const Matrix< T, M, N >& lhs,
				const Matrix< T, M, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	for(std::size_t row = 0;row<M;row++)
	{
		for(std::size_t column = 0;column<N;column++)
		{
			if(lhs[row][column]-aPrecision*aFactor>rhs[row][column]||lhs[row][column]+aPrecision*aFactor<rhs[row][column])
			{
				return false;
			}
		}
	}
	return true;
}
