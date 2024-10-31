#ifndef MAINSETTINGS_HPP_
#define MAINSETTINGS_HPP_

#include "Config.hpp"

namespace Application
{

	/*
	 *
	 */
	class MainSettings
	{
		public:
			/**
			 *
			 */
			MainSettings();
			/**
			 *
			 */
			virtual ~MainSettings();
			/**
			 *
			 */
			bool getDrawOpenSet() const;
			/**
			 *
			 */
			void setDrawOpenSet( bool aDrawOpenSet);
			/**
			 *
			 */
			unsigned long getSpeed() const;
			/**
			 *
			 */
			void setSpeed( unsigned long aSpeed);
			/**
			 *
			 */
			unsigned long getWorldNumber() const;
			/**
			 *
			 */
			void setWorldNumber( unsigned long aWorldNumber);
			/**
			 *
			 */
			unsigned long getFilterType() const;
			/**
			 *
			 */
			void setFilterType( unsigned long aFilterType);

		private:
			bool drawOpenSet;
			unsigned long speed;
			unsigned long worldNumber;
			unsigned long filterType;
	};

} /* namespace Application */

#endif /* SRC_MAINSETTINGS_HPP_ */
