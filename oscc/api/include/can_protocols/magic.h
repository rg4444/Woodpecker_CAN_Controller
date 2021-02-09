/**
 * @file magic.h
 * @brief Definitions of the magic bytes identifying messages as from OSCC.
 *
 */


#ifndef _OSCC_MAGIC_H_
#define _OSCC_MAGIC_H_


/*
 * @brief First magic byte used in commands and reports to distinguish CAN
 *        frame as coming from OSCC (and not OBD).
 *
 */
#define OSCC_MAGIC_BYTE_0 ( 0x05 )

/*
 * @brief Second magic byte used in commands and reports to distinguish CAN
 *        frame as coming from OSCC (and not OBD).
 *
 */
#define OSCC_MAGIC_BYTE_1 ( 0xCC )

#define OSCC_DRIVE_0 ( 0x00 )
#define OSCC_DRIVE_1 ( 0x01 )
#define OSCC_DRIVE_2 ( 0x02 )
#define OSCC_DRIVE_3 ( 0x03 )
#define OSCC_DRIVE_4 ( 0x04 )

#endif /* _OSCC_MAGIC_H_ */
