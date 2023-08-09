complete -c cyme -s d -l vidpid -d 'Show only devices with the specified vendor and product ID numbers (in hexadecimal) in format VID:[PID]' -r
complete -c cyme -s s -l show -d 'Show only devices with specified device and/or bus numbers (in decimal) in format [[bus]:][devnum]' -r
complete -c cyme -s D -l device -d 'Selects which device lsusb will examine - supplied as Linux /dev/bus/usb/BBB/DDD style path' -r
complete -c cyme -l filter-name -d 'Filter on string contained in name' -r
complete -c cyme -l filter-serial -d 'Filter on string contained in serial' -r
complete -c cyme -l filter-class -d 'Filter on USB class code' -r -f -a "{use-interface-descriptor	Device class is unspecified\, interface descriptors are used to determine needed drivers,audio	Speaker\, microphone\, sound card\, MIDI,cdc-communications	The modern serial interface; appears as a UART/RS232 port on most systems,hid	Human Interface Device; game controllers\, keyboards\, mice etc. Also commonly used as a device data interface rather then creating something from scratch,physical	Force feedback joystick,image	Scanners\, cameras,printer	Laser printer\, inkjet printer\, CNC machine,mass-storage	Mass storage devices (MSD): USB flash drive\, memory card reader\, digital audio player\, digital camera\, external drive,hub	High speed USB hub,cdc-data	Used together with class 02h (Communications and CDC Control) above,smart-cart	USB smart card reader,content-security	Fingerprint reader,video	Webcam,personal-healthcare	Pulse monitor (watch),audio-video	Webcam\, TV,billboard	Describes USB-C alternate modes supported by device,usb-type-c-bridge	An interface to expose and configure the USB Type-C capabilities of Connectors on USB Hubs or Alternate Mode Adapters,i3c-device	An interface to expose and configure I3C function within a USB device to allow interaction between host software and the I3C device\, to drive transaction on the I3C bus to/from target devices,diagnostic	Trace and debugging equipment,wireless-controller	Wireless controllers: Bluetooth adaptors\, Microsoft RNDIS,miscellaneous	This base class is defined for miscellaneous device definitions. Some matching SubClass and Protocols are defined on the USB-IF website,application-specific-interface	This base class is defined for devices that conform to several class specifications found on the USB-IF website,vendor-specific-class	This base class is defined for vendors to use as they please}"
complete -c cyme -s b -l blocks -d 'Specify the blocks which will be displayed for each device and in what order' -r -f -a "{bus-number	Number of bus device is attached,device-number	Bus issued device number,branch-position	Position of device in parent branch,port-path	Linux style port path,sys-path	Linux udev reported syspath,driver	Linux udev reported driver loaded for device,icon	Icon based on VID/PID,vendor-id	Unique vendor identifier - purchased from USB IF,product-id	Vendor unique product identifier,name	The device name as reported in descriptor or using usb_ids if None,manufacturer	The device manufacturer as provided in descriptor or using usb_ids if None,product-name	The device product name as reported by usb_ids vidpid lookup,vendor-name	The device vendor name as reported by usb_ids vid lookup,serial	Device serial string as reported by descriptor,speed	Advertised device capable speed,tree-positions	Position along all branches back to trunk device,bus-power	macOS system_profiler only - actually bus current in mA not power!,bus-power-used	macOS system_profiler only - actually bus current used in mA not power!,extra-current-used	macOS system_profiler only - actually bus current used in mA not power!,bcd-device	The device version,bcd-usb	The supported USB version,class-code	Class of interface provided by USB IF - only available when using libusb,sub-class	Sub-class of interface provided by USB IF - only available when using libusb,protocol	Prototol code for interface provided by USB IF - only available when using libusb}"
complete -c cyme -l bus-blocks -d 'Specify the blocks which will be displayed for each bus and in what order' -r -f -a "{bus-number	System bus number identifier,icon	Icon based on VID/PID,name	Bus name from descriptor or usb_ids,host-controller	Host Controller on macOS\, vendor put here when using libusb,pci-vendor	Understood to be vendor ID - it is when using libusb,pci-device	Understood to be product ID - it is when using libusb,pci-revision	Revsision of hardware,port-path	syspath style port path to bus\, applicable to Linux only}"
complete -c cyme -l config-blocks -d 'Specify the blocks which will be displayed for each configuration and in what order' -r -f -a "{name	Name from string descriptor,number	Number of config\, bConfigurationValue; value to set to enable to configuration,num-interfaces	Interfaces available for this configuruation,attributes	Attributes of configuration\, bmAttributes,icon-attributes	Icon representation of bmAttributes,max-power	Maximum current consumption in mA}"
complete -c cyme -l interface-blocks -d 'Specify the blocks which will be displayed for each interface and in what order' -r -f -a "{name	Name from string descriptor,number	Interface number,port-path	Interface port path\, applicable to Linux,class-code	Class of interface provided by USB IF,sub-class	Sub-class of interface provided by USB IF,protocol	Prototol code for interface provided by USB IF,alt-setting	Interfaces can have the same number but an alternate settings defined here,driver	Driver obtained from udev on Linux only,sys-path	syspath obtained from udev on Linux only,num-endpoints	An interface can have many endpoints,icon	Icon based on ClassCode/SubCode/Protocol}"
complete -c cyme -l endpoint-blocks -d 'Specify the blocks which will be displayed for each endpoint and in what order' -r -f -a "{number	Endpoint number on interface,direction	Direction of data into endpoint,transfer-type	Type of data transfer endpoint accepts,sync-type	Synchronisation type (Iso mode),usage-type	Usage type (Iso mode),max-packet-size	Maximum packet size in bytes endpoint can send/recieve,interval	Interval for polling endpoint data transfers. Value in frame counts. Ignored for Bulk & Control Endpoints. Isochronous must equal 1 and field may range from 1 to 255 for interrupt endpoints}"
complete -c cyme -l sort-devices -d 'Sort devices by value' -r -f -a "{branch-position	Sort by position in parent branch,device-number	Sort by bus device number,no-sort	No sorting; whatever order it was parsed}"
complete -c cyme -l group-devices -d 'Group devices by value when listing' -r -f -a "{no-group	No grouping,bus	Group into buses with bus info as heading - like a flat tree}"
complete -c cyme -l color -d 'Output coloring mode' -r -f -a "{auto	Show colours if the output goes to an interactive console,always	Always apply colouring to the output,never	Never apply colouring to the output}"
complete -c cyme -l encoding -d 'Output charactor encoding' -r -f -a "{glyphs	Use UTF-8 private use area charactors such as those used by NerdFont to show glyph icons,utf8	Use only standard UTF-8 charactors for the output; no private use area glyph icons,ascii	Use only ASCII charactors for the output; 0x00 - 0x7F (127 chars)}"
complete -c cyme -l icon -d 'When to print icon blocks' -r -f -a "{auto	Show icon blocks if the [`Encoding`] supports icons matched in the [`icon::IconTheme`],always	Always print icon blocks if included in configured blocks,never	Never print icon blocks}"
complete -c cyme -l from-json -d 'Read from json output rather than profiling system - must use --tree json dump' -r
complete -c cyme -s c -l config -d 'Path to user config file to use for custom icons, colours and default settings' -r
complete -c cyme -l mask-serials -d 'Mask serial numbers with \'*\' or random chars' -r -f -a "{hide	Hide with \'*\' char,scramble	Mask by randomising existing chars,replace	Mask by replacing length with random chars}"
complete -c cyme -s l -l lsusb -d 'Attempt to maintain compatibility with lsusb output'
complete -c cyme -s t -l tree -d 'Dump USB device hierarchy as a tree'
complete -c cyme -s v -l verbose -d 'Verbosity level: 1 prints device configurations; 2 prints interfaces; 3 prints interface endpoints; 4 prints everything and all blocks'
complete -c cyme -s m -l more -d 'Print more blocks by default at each verbosity'
complete -c cyme -l sort-buses -d 'Sort devices by bus number'
complete -c cyme -l hide-buses -d 'Hide empty buses when printing tree; those with no devices. When listing will hide Linux root_hubs'
complete -c cyme -l hide-hubs -d 'Hide empty hubs when printing tree; those with no devices. When listing will hide hubs regardless of whether empty of not'
complete -c cyme -l decimal -d 'Show base16 values as base10 decimal instead'
complete -c cyme -l no-padding -d 'Disable padding to align blocks - will cause --headings to become maligned'
complete -c cyme -l no-color -d 'Disable coloured output, can also use NO_COLOR environment variable'
complete -c cyme -l ascii -d 'Disables icons and utf-8 charactors'
complete -c cyme -l no-icons -d 'Disables all Block icons by not using any IconTheme. Providing custom XxxxBlocks without any icons is a nicer way to do this'
complete -c cyme -l headings -d 'Show block headings'
complete -c cyme -l json -d 'Output as json format after sorting, filters and tree settings are applied; without -tree will be flattened dump of devices'
complete -c cyme -s F -l force-libusb -d 'Force libusb profiler on macOS rather than using/combining system_profiler output'
complete -c cyme -s z -l debug -d 'Turn debugging information on. Alternatively can use RUST_LOG env: INFO, DEBUG, TRACE'
complete -c cyme -l gen -d 'Generate cli completions and man page'
complete -c cyme -s h -l help -d 'Print help (see more with \'--help\')'
complete -c cyme -s V -l version -d 'Print version'
