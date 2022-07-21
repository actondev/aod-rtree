(defmacro aod-rtree/defn (name &rest body)
  (let ((root (projectile-project-root) ))
    `(defun ,(intern (format "aod-rtree/%s" name))
	 ()
       (interactive)
       (let ((default-directory ,root))
	 ,@body))))

(aod-rtree/defn compile
		(compile "meson compile -C build # --verbose"))

(aod-rtree/defn benchmark
		(compile "meson test --benchmark -C build -v"))

(aod-rtree/defn test
		(let ((debug (aod.transient/flag 'aod-rtree "--debug")))
		  (if debug
		      (s7ty/debug-weirdNox-gdb "build/test/run-tests" "")
		    (test-simple-run "meson test -C build -v"))))

(defun aod-rtree/debug-weirdNox-gdb (command args)
  (gdb-executable command) ; nox fork ;;-i=mi
  (gdb--command (format "-exec-arguments %s" args))
  (gdb--command "-exec-run --start"))

(transient-define-prefix aod-rtree ()
  ["rtree"
   ("q" "quit" transient-quit-all)
   ("Q" "quit & restore windows" aod-do/restore :transient transient--do-quit-all)]
  ["Args"
   ("-d" "debug" "--debug")]
  ["Commands"
   ("c" "compile" aod-rtree/compile ; :transient transient--do-call
    )
   ("t" "test" aod-rtree/test)
   ("b" "benchmark" aod-rtree/benchmark)
   ;;
   ])

(setq-local aod-do/action #'aod-rtree)

(comment "custom actions"
	 
	 (aod-do/register-action #'aod-rtree)
	 )
