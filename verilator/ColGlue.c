#define MAX_DISKS     4       /* Maximal number of disks     */
#define MAX_TAPES     4       /* Maximal number of tapes     */
#include "EMULib/FDIDisk.h"          /* Disk images                 */

byte Verbose     = 1;          /* Debug msgs ON/OFF             */
FDIDisk Disks[MAX_DISKS] = { 0 };  /* Adam disk drives          */
FDIDisk Tapes[MAX_TAPES] = { 0 };  /* Adam tape drives          */
